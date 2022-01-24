//Corresponding header
#include "robo_collector_gui/helpers/TurnHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t TurnHelper::init(const TurnHelperConfig& cfg) {
  if (nullptr == cfg.finishPlayerActCb) {
    LOGERR("Error, nullptr provided for finishPlayerActCb");
    return FAILURE;
  }
  _finishPlayerActCb = cfg.finishPlayerActCb;

  return SUCCESS;
}

void TurnHelper::onRobotFinishAct(int32_t robotId) {
  if (Defines::BLINKY_IDX == robotId) {
    _finishPlayerActCb();
  }
}
