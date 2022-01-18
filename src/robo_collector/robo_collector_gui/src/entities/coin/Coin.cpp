//Corresponding header
#include "robo_collector_gui/entities/coin/Coin.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/field/FieldUtils.h"

int32_t Coin::init(const CoinConfig& cfg) {
  AnimBaseConfig animCfg;
  animCfg.rsrcId = cfg.rsrcId;
  animCfg.timerId = cfg.timerId;
  animCfg.timerInterval = 75;
  animCfg.startPos = FieldUtils::getAbsPos(cfg.fieldPos);
  animCfg.startPos += cfg.tileOffset;
  animCfg.animDirection = AnimDir::FORWARD;

  if (SUCCESS != _anim.configure(animCfg)) {
    LOGERR("Coin anim configure failled for rsrcId: %#16lX", cfg.rsrcId);
    return FAILURE;
  }

  _anim.start();

  return SUCCESS;
}

void Coin::draw() const {
  _anim.draw();
}

