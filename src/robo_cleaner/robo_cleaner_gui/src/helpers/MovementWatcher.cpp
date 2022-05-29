//Corresponding header
#include "robo_cleaner_gui/helpers/MovementWatcher.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers

ErrorCode MovementWatcher::init(
    const ReportMoveProgressCb &reportMoveProgressCb) {
  if (nullptr == reportMoveProgressCb) {
    LOGERR("Error, nullptr received for ReportMoveProgressCb");
    return ErrorCode::FAILURE;
  }
  _reportMoveProgressCb = reportMoveProgressCb;

  return ErrorCode::SUCCESS;
}

void MovementWatcher::process() {

}

void MovementWatcher::changeState([[maybe_unused]]const RobotState &state,
                                  MoveOutcome outcome) {
  _currProgress.outcome = outcome;
  _currProgress.hasMoveFinished = true;

  _reportMoveProgressCb(_currProgress);
  _currProgress.reset();
}

