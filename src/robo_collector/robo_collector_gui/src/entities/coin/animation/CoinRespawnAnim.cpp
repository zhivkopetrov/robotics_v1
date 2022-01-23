//Corresponding header
#include "robo_collector_gui/entities/coin/animation/CoinRespawnAnim.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/field/FieldPos.h"

namespace {
constexpr auto ANIM_STEPS = 6;
}

int32_t CoinRespawnAnim::init(const CoinRespawnAnimConfig& cfg) {
  if (nullptr == cfg.animEndCb) {
    LOGERR("Error, nullptr provided for CoinRespawnAnim coinAnimEndCb");
    return FAILURE;
  }
  _animEndCb = cfg.animEndCb;

  if (nullptr == cfg.coinImg) {
    LOGERR("Error, nullptr provided for coinImg");
    return FAILURE;
  }
  _coinImg = cfg.coinImg;
  _timerId = cfg.timerId;
  animationSteps = ANIM_STEPS;

  return SUCCESS;
}

void CoinRespawnAnim::start() {
  constexpr auto timerInterval = 250;
  startTimer(timerInterval, _timerId, TimerType::PULSE);
}

void CoinRespawnAnim::onTimeout(const int32_t timerId) {
  if (timerId == _timerId) {
    processAnim();
  } else {
    LOGERR("Error, received unsupported timerId: %d", timerId);
  }
}

void CoinRespawnAnim::processAnim() {
  _coinImg->isVisible() ? _coinImg->hide() : _coinImg->show();
  --animationSteps;
  if (0 == animationSteps) {
    stopTimer(_timerId);
    animationSteps = ANIM_STEPS;
    _animEndCb(CoinAnimType::RESPAWN);
  }
}
