//Corresponding header
#include "robo_common/layout/entities/robot/animation/RobotAnimEndCb.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/log/Log.h"

//Own components headers

ErrorCode RobotAnimEndCb::init(
    const std::function<void(Direction, const FieldPos&)> &onRobotsAnimEndCb) {
  if (nullptr == onRobotsAnimEndCb) {
    LOGERR("Error, nullptr provided for onRobotsAnimEndCb");
    return ErrorCode::FAILURE;
  }
  _onRobotsAnimEndCb = onRobotsAnimEndCb;

  return ErrorCode::SUCCESS;
}

ErrorCode RobotAnimEndCb::onAnimationEnd() {
  if (RobotAnimEndCbReport::ENABLE ==_status) {
    _onRobotsAnimEndCb(_futureDir, _futurePos);
  }

  return ErrorCode::SUCCESS;
}

void RobotAnimEndCb::setAnimEndData(Direction futureDir,
                                    const FieldPos &futurePos) {
  _futureDir = futureDir;
  _futurePos = futurePos;
}

void RobotAnimEndCb::setCbStatus(RobotAnimEndCbReport status) {
  _status = status;
}
