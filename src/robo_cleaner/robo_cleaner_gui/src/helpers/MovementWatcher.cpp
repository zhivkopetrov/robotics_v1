//Corresponding header
#include "robo_cleaner_gui/helpers/MovementWatcher.h"

//System headers

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "robo_common/layout/entities/robot/helpers/RobotUtils.h"
#include "sdl_utils/drawing/defines/DrawConstants.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/data_type/FloatingPointUtils.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/helpers/RoboCleanerSolutionValidator.h"

ErrorCode MovementWatcher::init(MovementWatcherConfig &cfg,
                                const MovementWatcherOutInterface &interface) {
  _cfg = cfg;
  if (ErrorCode::SUCCESS != initOutInterface(interface)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void MovementWatcher::process() {
  if (!_isFeedbackReportActive) {
    return;
  }

  if (MoveType::FORWARD == _currMoveType) {
    processForwardMovement();
  } else if ( (MoveType::ROTATE_LEFT == _currMoveType)
      || (MoveType::ROTATE_RIGHT == _currMoveType)) {
    processRotationMovement();
  } else {
    LOGERR("Error, bad state for MoveType: %d", getEnumValue(_currMoveType));
    return;
  }
}

void MovementWatcher::onRobotStartingAct(MoveType moveType,
                                         char approachingMarker) {
  _currProgress.approachingFieldMarker =
      static_cast<uint8_t>(approachingMarker);
  const RobotState robotState = _outInterface.getRobotStateCb();

  if (MoveType::FORWARD == moveType) {
    const FieldPos robotFieldPos = FieldUtils::getAdjacentPos(robotState.dir,
        robotState.fieldPos);
    _targetAbsolutePos = FieldUtils::getAbsPos(robotFieldPos,
        _outInterface.getFieldDescriptionCb());
  } else if (MoveType::ROTATE_LEFT == moveType) {
    _currRotationDir = RotationDir::LEFT;
    const Direction dir = RobotUtils::getDirAfterRotation(robotState.dir,
        _currRotationDir);
    _targetRotation = RobotUtils::getRotationDegFromDir(dir);
  } else if (MoveType::ROTATE_RIGHT == moveType) {
    _currRotationDir = RotationDir::RIGHT;
    const Direction dir = RobotUtils::getDirAfterRotation(robotState.dir,
        _currRotationDir);
    _targetRotation = RobotUtils::getRotationDegFromDir(dir);
    if (FloatingPointUtils::areAlmostEqual(ZERO_ANGLE, _targetRotation)) {
      _targetRotation = FULL_ROTATION_ANGLE;
    }
  } else {
    LOGERR("Error, bad state for MoveType: %d", getEnumValue(moveType));
    return;
  }

  _currMoveType = moveType;
  _isFeedbackReportActive = true;
}

void MovementWatcher::onInsufficientEnergy(int32_t penaltyMoves) {
  LOGR("Insufficient energy to perform a move. Restoring 50 %% battery after "
      "[%d] applied penalty moves", penaltyMoves);

  _currProgress.hasEnergyForMovement = false;
  _outInterface.reportMoveProgressCb(_currProgress);
  reset();
}

void MovementWatcher::onObstacleApproachTrigger(const FieldPos &fieldPos) {
  const auto &fieldDescr = _outInterface.getFieldDescriptionCb();
  if (FieldUtils::isInsideField(fieldPos, fieldDescr)) {
    _currProgress.approachingFieldMarker =
        static_cast<uint8_t>(fieldDescr.data[fieldPos.row][fieldPos.col]);
  } else {
    //map surrounding obstacles are outside of field
    _currProgress.approachingFieldMarker =
        RoboCommonDefines::FIELD_OUT_OF_BOUND_MARKER;
  }

  process(); //send an immediate feedback update
}

void MovementWatcher::changeState(const RobotState &state,
                                  MoveOutcome outcome) {
  _currProgress.outcome = outcome;
  _currProgress.hasMoveFinished = true;

  const auto [tileRevealed, tileCleaned, processedMarker, reachedEndGameCond] =
      _outInterface.solutionValidator->finishMove(
          state, outcome, _currMoveType);
  _currProgress.processedFieldMarker = processedMarker;

  if (tileRevealed) {
    _outInterface.tileReleavedCb();
  }

  if (tileCleaned) {
    _outInterface.setFieldDataMarkerCb(state.fieldPos, processedMarker);
    _outInterface.modifyRubbishWidgetCb(state.fieldPos, processedMarker);
    _outInterface.tileCleanedCb();
  }

  if (reachedEndGameCond) {
    const int32_t robotHealthValue =
        _outInterface.getRobotHealthIndicatorValueCb();
    const bool isAtFullHealth = INDICATOR_PANEL_MAX_VALUE == robotHealthValue;
    if (isAtFullHealth) {
      _outInterface.startAchievementWonAnimCb(Achievement::TRIPLE_STAR);
    }
    _outInterface.startGameWonAnimCb();
  }

  _outInterface.reportMoveProgressCb(_currProgress);
  reset();
}

void MovementWatcher::cancelFeedbackReporting() {
  _isFeedbackReportActive = false;

  //report move progress to wake the reported thread
  //the reported thread should then detect that the goal has been cancelled
  //by the action client and will cancel it for the action server as well
  _outInterface.reportMoveProgressCb(_currProgress);
  reset();
}

ErrorCode MovementWatcher::initOutInterface(
    const MovementWatcherOutInterface &interface) {
  _outInterface = interface;
  if (nullptr == _outInterface.reportMoveProgressCb) {
    LOGERR("Error, nullptr provided for ReportMoveProgressCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.getRobotStateCb) {
    LOGERR("Error, nullptr provided for GetRobotStateCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.getRobotAbsolutePosCb) {
    LOGERR("Error, nullptr provided for GetRobotAbsolutePosCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.setFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for SetFieldDataMarkerCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.modifyRubbishWidgetCb) {
    LOGERR("Error, nullptr provided for ModifyRubbishWidgetCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.tileReleavedCb) {
    LOGERR("Error, nullptr provided for TileReleavedCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.tileCleanedCb) {
    LOGERR("Error, nullptr provided for TileCleanedCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.startGameWonAnimCb) {
    LOGERR("Error, nullptr provided for StartGameWonAnimCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.startAchievementWonAnimCb) {
    LOGERR("Error, nullptr provided for StartAchievementWonAnimCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.getRobotHealthIndicatorValueCb) {
    LOGERR("Error, nullptr provided for GetIndicatorPanelValue");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.solutionValidator) {
    LOGERR("Error, nullptr provided for SolutionValidator");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void MovementWatcher::processForwardMovement() {
  const Point currAbsolutePos = _outInterface.getRobotAbsolutePosCb();
  int32_t progressToReport { };

  if (_targetAbsolutePos.x == currAbsolutePos.x) { //moving on Y axis
    const int32_t remainingAbsoluteProgress = std::abs(
        _targetAbsolutePos.y - currAbsolutePos.y);
    const int32_t remainingProgressPercent =
        static_cast<int32_t>( (remainingAbsoluteProgress * 100.0)
            / _cfg.tileHeight);
    progressToReport = 100 - remainingProgressPercent;
  } else { //moving on X axis
    const int32_t remainingAbsoluteProgress = std::abs(
        _targetAbsolutePos.x - currAbsolutePos.x);
    const int32_t remainingProgressPercent =
        static_cast<int32_t>( (remainingAbsoluteProgress * 100.0)
            / _cfg.tileWidth);
    progressToReport = 100 - remainingProgressPercent;
  }

  // currently the feedback pool loop is tied to the GUI main loop
  // Since GUI tied timers operate at a lower interval than the main loop -
  // on some main loop updates there will not be an updates on the TimerClient
  // Don't, report feedback in that scenario, because it will be the same as
  // the previous one
  if (_currProgress.progress != progressToReport) {
    _currProgress.progress = progressToReport;
    _outInterface.reportMoveProgressCb(_currProgress);
  }
}

void MovementWatcher::processRotationMovement() {
  int32_t progressToReport { };

  constexpr double totalRotation = 90.0;
  const double currAbsoluteRotation = [this]() {
    double absoluteRotation = _outInterface.getRobotRotationAngleCb();
    if (RotationDir::LEFT == _currRotationDir) {
      if (FloatingPointUtils::areAlmostEqual(ZERO_ANGLE, absoluteRotation)) {
        absoluteRotation = FULL_ROTATION_ANGLE;
      }
    } else {
      if (FloatingPointUtils::areAlmostEqual(FULL_ROTATION_ANGLE,
          absoluteRotation)) {
        absoluteRotation = ZERO_ANGLE;
      }
    }
    return absoluteRotation;
  }();

  const double remainingAbsoluteProgress = std::fabs(
      _targetRotation - currAbsoluteRotation);
  const int32_t remainingProgressPercent =
      static_cast<int32_t>( (remainingAbsoluteProgress * 100) / totalRotation);
  progressToReport = 100 - remainingProgressPercent;

  // currently the feedback pool loop is tied to the GUI main loop
  // Since GUI tied timers operate at a lower interval than the main loop -
  // on some main loop updates there will not be an updates on the TimerClient
  // Don't, report feedback in that scenario, because it will be the same as
  // the previous one
  if (_currProgress.progress != progressToReport) {
    _currProgress.progress = progressToReport;
    _outInterface.reportMoveProgressCb(_currProgress);
  }
}

void MovementWatcher::reset() {
  _currProgress.reset();
  _targetAbsolutePos = Points::UNDEFINED;
  _isFeedbackReportActive = false;
  _targetRotation = 0.0;
}

