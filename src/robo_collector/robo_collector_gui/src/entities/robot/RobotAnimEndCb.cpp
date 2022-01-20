//Corresponding header
#include "robo_collector_gui/entities/robot/RobotAnimEndCb.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t RobotAnimEndCb::init(
    const std::function<void(Direction, const FieldPos&)> &robotSetDataCb) {
  if (nullptr == robotSetDataCb) {
    LOGERR("Error, nullptr provided for robotSetDataCb");
    return FAILURE;
  }
  _robotSetDataCb = robotSetDataCb;

  return SUCCESS;
}

int32_t RobotAnimEndCb::onAnimationEnd() {
  _robotSetDataCb(_futureDir, _futurePos);
  return SUCCESS;
}

void RobotAnimEndCb::setAnimEndData(Direction futureDir,
                                    const FieldPos &futurePos) {
  _futureDir = futureDir;
  _futurePos = futurePos;
}
