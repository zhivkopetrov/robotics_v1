//Corresponding header
#include "ur_control_bloom/motion/JengaMotionSequence.h"

//System headers

//Other libraries headers
#include "urscript_common/motion/UrScriptBuilder.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

JengaMotionSequence::JengaMotionSequence(
  const JengaMotionSequenceConfig& cfg,
  const std::string& name, int32_t id) : MotionSequence(name, id), _cfg(cfg) {
  populateUrscriptHeaders();
}

void JengaMotionSequence::start(const MotionCommandBatchDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Jenga::GRASP_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Jenga::TRANSPORT_AND_PLACE_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void JengaMotionSequence::gracefulStop(const MotionCommandBatchDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING }
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void JengaMotionSequence::recover(const MotionCommandBatchDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Jenga::TRANSPORT_AND_PLACE_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void JengaMotionSequence::populateUrscriptHeaders() {
  //TODO parse from files
  const AngleAxis orientation(0.0, -3.16, 0.0);
  constexpr double accel = 1.0;
  constexpr double vel = 1.0;
  constexpr double blendingRadius = 0.0;

  const WaypointCartesian graspPose(Point3d(-0.5, -0.6, 0.2), orientation);
  auto graspCommand = std::make_unique<MoveLinearCommand>(
    graspPose, accel, vel, blendingRadius);

  const WaypointCartesian transportPose(Point3d(0.0, -0.6, 0.2), orientation);
  auto transportCommand = std::make_unique<MoveLinearCommand>(
    transportPose, accel, vel, blendingRadius);

  const WaypointCartesian homePose(Point3d(0.5, -0.4, 0.6), orientation);
  auto returnHomeCommand = std::make_unique<MoveLinearCommand>(
    homePose, accel, vel, blendingRadius);

  MotionCommandContainer cmdContainer;

  cmdContainer.addCommand(std::move(graspCommand));
  urScriptHeaders[Motion::Jenga::GRASP_NAME] = UrScriptBuilder::construct(
    Motion::Jenga::GRASP_NAME, cmdContainer);

  cmdContainer.addCommand(std::move(transportCommand));
  urScriptHeaders[Motion::Jenga::TRANSPORT_AND_PLACE_NAME] = 
    UrScriptBuilder::construct(
      Motion::Jenga::TRANSPORT_AND_PLACE_NAME, cmdContainer);

  cmdContainer.addCommand(std::move(returnHomeCommand));
  urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME] = UrScriptBuilder::construct(
    Motion::Jenga::RETURN_HOME_NAME, cmdContainer);
}