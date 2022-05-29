//Corresponding header
#include "robo_cleaner_gui/external_api/MovementReporter.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers

ErrorCode MovementReporter::init(
    const ResetControllerStatusCb& resetControllerStatusCb) {
  if (nullptr == resetControllerStatusCb) {
    LOGERR("Error, received nullptr for ResetControllerStatusCb");
    return ErrorCode::FAILURE;
  }
  _resetControllerStatusCb = resetControllerStatusCb;

  _thread = std::thread([this](){
    run();
  });

  return ErrorCode::SUCCESS;
}

void MovementReporter::deinit() {
  _activeGoals.shutdown();
  _progressReports.shutdown();
  _thread.join();
}

void MovementReporter::acceptGoal(
    const std::shared_ptr<GoalHandleRobotMove> &goalHandle) {
  // return quickly to avoid blocking the executor
  _activeGoals.pushWithCopy(goalHandle);
}

void MovementReporter::reportProgress(const MoveProgress &progress) {
  _progressReports.pushWithCopy(progress);
}

void MovementReporter::run() {
  while (true) {
    std::shared_ptr<GoalHandleRobotMove> goalHandle;
    const auto [isShutdowned, hasTimedOut] =
        _activeGoals.waitAndPop(goalHandle);
    if (isShutdowned) {
      return;
    }
    if (hasTimedOut) {
      continue;
    }

    reportProgressLoop(goalHandle);
  }
}

void MovementReporter::reportProgressLoop(
    const std::shared_ptr<GoalHandleRobotMove> &goalHandle) {
  auto feedback = std::make_shared<RobotMove::Feedback>();
  auto result = std::make_shared<RobotMove::Result>();

  MoveProgress moveProgress;
  while (true) {
    const auto [isShutdowned, hasTimedOut] = _progressReports.waitAndPop(
        moveProgress);
    if (isShutdowned) {
      return;
    }
    if (hasTimedOut) {
      continue;
    }

    if (goalHandle->is_canceling()) {
      result->success = false;
      result->error_reason = "Goal with uuid: %s was cancelled",
          rclcpp_action::to_string(goalHandle->get_goal_id()).c_str();
      goalHandle->canceled(result);
      LOGR("%s", result->error_reason.c_str());

      _resetControllerStatusCb();
      return;
    }

    if (moveProgress.hasMoveFinished) {
      if (MoveOutcome::SUCCESS == moveProgress.outcome) {
        result->success = true;
        result->processed_field_marker = moveProgress.processedFieldMarker;
        goalHandle->succeed(result);
        LOGG("Goal with uuid: %s succeeded",
            rclcpp_action::to_string(goalHandle->get_goal_id()).c_str());
      } else {
        result->success = false;
        result->error_reason = "Move resulted in collision";
        goalHandle->succeed(result);
        LOGR("Goal with uuid: %s resulted in collision",
            rclcpp_action::to_string(goalHandle->get_goal_id()).c_str());
      }

      _resetControllerStatusCb();
      return;
    }

    feedback->progress_percent = moveProgress.progress;
    feedback->approaching_field_marker = moveProgress.approachingFieldMarker;
    goalHandle->publish_feedback(feedback);
  }
}

