//Corresponding header
#include "robo_collector_gui/layout/entities/coin/animation/CoinRespawnAnim.h"

//System headers

//Other libraries headers
#include "utils/log/Log.h"

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
  //the game is going too fast. This is not a logical error,
  //but animation needs to be updated accordingly
  //This is due to the fact that the coin was collided while still
  //in respawn phase
  //the new position is already set -> keep the animation active and reset state
  if (isAnimationActive()) {
    animationSteps = ANIM_STEPS;
    restartTimerInterval(_timerId);
    return;
  }

  constexpr auto timerInterval = 250;
  startTimer(timerInterval, _timerId, TimerType::PULSE);
}

bool CoinRespawnAnim::isAnimationActive() const {
  return isActiveTimerId(_timerId);
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
