//Corresponding header
#include "robo_collector_gui/layout/entities/coin/animation/CoinRespawnAnim.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto ANIM_STEPS = 6;
}

ErrorCode CoinRespawnAnim::init(const CoinRespawnAnimConfig& cfg) {
  if (nullptr == cfg.animEndCb) {
    LOGERR("Error, nullptr provided for CoinRespawnAnim coinAnimEndCb");
    return ErrorCode::FAILURE;
  }
  _animEndCb = cfg.animEndCb;

  if (nullptr == cfg.coinImg) {
    LOGERR("Error, nullptr provided for coinImg");
    return ErrorCode::FAILURE;
  }
  _coinImg = cfg.coinImg;
  _timerId = cfg.timerId;
  animationSteps = ANIM_STEPS;

  return ErrorCode::SUCCESS;
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
