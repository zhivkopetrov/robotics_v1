//Corresponding header
#include "robo_collector_gui/entities/coin/Coin.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/field/FieldUtils.h"

namespace {
constexpr auto TARGET_COLLECT_ANIM_X = 1285;
constexpr auto TARGET_COLLECT_ANIM_Y = 243;
}

int32_t Coin::init(const CoinConfig &cfg) {
  collectAnimTimerId = cfg.collectAnimTimerId;
  _coinImg.create(cfg.rsrcId);

  AnimBaseConfig animCfg;
  animCfg.timerId = cfg.rotateAnimTimerId;
  animCfg.timerInterval = 75;
  animCfg.startPos = FieldUtils::getAbsPos(cfg.fieldPos);
  animCfg.startPos += cfg.tileOffset;
  animCfg.animDirection = AnimDir::FORWARD;
  animCfg.animImageType = AnimImageType::EXTERNAL;
  animCfg.externalImage = &_coinImg;

  if (SUCCESS != _rotateAnim.configure(animCfg)) {
    LOGERR("Coin _rotateAnim configure failed for rsrcId: %#16lX", cfg.rsrcId);
    return FAILURE;
  }

  _rotateAnim.start();

  return SUCCESS;
}

void Coin::draw() const {
  _coinImg.draw();
}

void Coin::startCollectAnim() {
  AnimBaseConfig cfg;
  cfg.timerId = collectAnimTimerId;
  cfg.timerInterval = 20;
  cfg.startPos = _coinImg.getPosition();
  cfg.animDirection = AnimDir::FORWARD;
  cfg.animImageType = AnimImageType::EXTERNAL;
  cfg.externalImage = &_coinImg;
  const Point endPos = Point(TARGET_COLLECT_ANIM_X, TARGET_COLLECT_ANIM_Y);
  const auto numberOfSteps = 50;
  AnimationEndCb *animEndCb = nullptr;

  if (SUCCESS != _posAnim.configure(cfg, endPos, numberOfSteps, animEndCb,
          PosAnimType::ONE_DIRECTIONAL)) {
    LOGERR("Error, _posAnim.configure() failed for rsrcId: %#16lX");
  }
  _posAnim.start();
}

