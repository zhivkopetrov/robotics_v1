//Corresponding header
#include "robo_collector_gui/entities/coin/Coin.h"

//C system headers

//C++ system headers
#include <cmath>

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/helpers/CollisionWatcher.h"
#include "robo_collector_gui/field/FieldUtils.h"

namespace {
constexpr auto TARGET_COLLECT_ANIM_X = 1285;
constexpr auto TARGET_COLLECT_ANIM_Y = 243;
}

int32_t Coin::init(const CoinConfig &cfg) {
  if (nullptr == cfg.collisionWatcher) {
    LOGERR("Error, nullptr provided for collisionWatcher");
    return FAILURE;
  }
  _collisionWatcher = cfg.collisionWatcher;
  _collisionObjHandle = cfg.collisionWatcher->registerObject(this);

  if (nullptr == cfg.incrCollectedCoinsCb) {
    LOGERR("Error, nullptr provided for CoinCfg incrCollectedCoinsCb");
    return FAILURE;
  }
  _incrCollectedCoinsCb = cfg.incrCollectedCoinsCb;

  if (SUCCESS !=
      _coinAnimEndCb.init(std::bind(&Coin::onCollectAnimEnd, this))) {
    LOGERR("Error, coinAnimEndCb.init() failed");
    return FAILURE;
  }

  _collectAnimTimerId = cfg.collectAnimTimerId;
  _coinScore = cfg.coinScore;
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

void Coin::deinit() {
  if (_collisionWatcher) {
    _collisionWatcher->unregisterObject(_collisionObjHandle);
  }
}

void Coin::draw() const {
  _coinImg.draw();
}

void Coin::onCollectAnimEnd() {
  _incrCollectedCoinsCb(_coinScore);
}

void Coin::startCollectAnim() {
  AnimBaseConfig cfg;
  cfg.timerId = _collectAnimTimerId;
  cfg.timerInterval = 20;
  cfg.startPos = _coinImg.getPosition();
  cfg.animDirection = AnimDir::FORWARD;
  cfg.animImageType = AnimImageType::EXTERNAL;
  cfg.externalImage = &_coinImg;
  const Point endPos = Point(TARGET_COLLECT_ANIM_X, TARGET_COLLECT_ANIM_Y);

  const auto deltaX = endPos.x - cfg.startPos.x;
  const auto deltaY = endPos.y - cfg.startPos.y;
  const auto distance =
      static_cast<int32_t>(sqrt((deltaX * deltaX) + (deltaY * deltaY)));
  const auto numberOfSteps = distance / 20;

  if (SUCCESS != _posAnim.configure(cfg, endPos, numberOfSteps, &_coinAnimEndCb,
          PosAnimType::ONE_DIRECTIONAL)) {
    LOGERR("Error, _posAnim.configure() failed for rsrcId: %#16lX");
  }
  _posAnim.start();
}

void Coin::registerCollision([[maybe_unused]]const Rectangle &intersectRect) {
  startCollectAnim();
}

Rectangle Coin::getBoundary() const {
  return _coinImg.getImageRect();
}

