//Corresponding header
#include "robo_collector_gui/entities/coin/Coin.h"

//C system headers

//C++ system headers
#include <cmath>

//Other libraries headers
#include "utils/rng/Rng.h"
#include "utils/data_type/EnumClassUtils.h"
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
  _collisionObjHandle =
      cfg.collisionWatcher->registerObject(this, CollisionDamageImpact::NO);

  if (nullptr == cfg.incrCollectedCoinsCb) {
    LOGERR("Error, nullptr provided for CoinCfg incrCollectedCoinsCb");
    return FAILURE;
  }
  _incrCollectedCoinsCb = cfg.incrCollectedCoinsCb;

  if (nullptr == cfg.setFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for CoinConfig setFieldDataMarkerCb");
    return FAILURE;
  }
  _setFieldDataMarkerCb = cfg.setFieldDataMarkerCb;

  if (nullptr == cfg.resetFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for CoinConfig resetFieldDataMarkerCb");
    return FAILURE;
  }
  _resetFieldDataMarkerCb = cfg.resetFieldDataMarkerCb;

  if (nullptr == cfg.getFieldDataCb) {
    LOGERR("Error, nullptr provided for CoinConfig getFieldDataCb");
    return FAILURE;
  }
  _getFieldDataCb = cfg.getFieldDataCb;

  const auto animEndCb =
      std::bind(&Coin::onAnimEnd, this, std::placeholders::_1);
  if (SUCCESS != _coinCollectAnimEndCb.init(animEndCb)) {
    LOGERR("Error, coinAnimEndCb.init() failed");
    return FAILURE;
  }

  CoinRespawnAnimConfig coinRespawnAnimCfg;
  coinRespawnAnimCfg.coinImg = &_coinImg;
  coinRespawnAnimCfg.timerId = cfg.respawnAnimTimerId;
  coinRespawnAnimCfg.animEndCb = animEndCb;
  if (SUCCESS != _respawnAnim.init(coinRespawnAnimCfg)) {
    LOGERR("Error, _respawnAnim.init() failed");
    return FAILURE;
  }

  _tileOffset = cfg.tileOffset;
  _collectAnimTimerId = cfg.collectAnimTimerId;
  _coinScore = cfg.coinScore;
  _fieldDataMarker = cfg.fieldDataMarker;
  _fieldEmptyDataMarker = cfg.fieldEmptyDataMarker;
  _coinImg.create(cfg.rsrcId);

  const auto fieldPos = choseRespawnLocation();

  AnimBaseConfig animCfg;
  animCfg.timerId = cfg.rotateAnimTimerId;
  animCfg.timerInterval = 75;
  animCfg.startPos = FieldUtils::getAbsPos(fieldPos);
  animCfg.startPos += cfg.tileOffset;
  animCfg.animDirection = AnimDir::FORWARD;
  animCfg.animImageType = AnimImageType::EXTERNAL;
  animCfg.externalImage = &_coinImg;

  _setFieldDataMarkerCb(fieldPos, _fieldDataMarker);

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

void Coin::onAnimEnd(CoinAnimType coinAnimType) {
  if (CoinAnimType::COLLECT == coinAnimType) {
    _incrCollectedCoinsCb(_coinScore);
    _resetFieldDataMarkerCb(FieldUtils::getFieldPos(_coinImg.getPosition()));
    const auto fieldPos = choseRespawnLocation();
    startRespawnAnim(fieldPos);
  } else if(CoinAnimType::RESPAWN == coinAnimType) {
    _setFieldDataMarkerCb(
        FieldUtils::getFieldPos(_coinImg.getPosition()), _fieldDataMarker);
  } else {
    LOGERR("Logical error, received wrong anim type: %d",
        getEnumValue(coinAnimType));
  }
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
  const auto pixelDistance =
      static_cast<int32_t>(sqrt((deltaX * deltaX) + (deltaY * deltaY)));
  constexpr auto pixelsPerStep = 20;
  const auto numberOfSteps = pixelDistance / pixelsPerStep;

  if (SUCCESS != _posAnim.configure(cfg, endPos, numberOfSteps,
          &_coinCollectAnimEndCb, PosAnimType::ONE_DIRECTIONAL)) {
    LOGERR("Error, _posAnim.configure() failed for rsrcId: %#16lX");
  }
  _posAnim.start();
}

void Coin::registerCollision([[maybe_unused]]const Rectangle &intersectRect,
                             [[maybe_unused]]CollisionDamageImpact impact) {
  startCollectAnim();
}

Rectangle Coin::getBoundary() const {
  return _coinImg.getImageRect();
}

FieldPos Coin::choseRespawnLocation() {
  const auto& fieldData = _getFieldDataCb();
  const auto cols = fieldData.size();
  const auto rows = fieldData[0].size();
  auto& rng = Rng::getInstance();
  FieldPos chosenPos;

  while (true) {
    chosenPos.row = rng.getRandomNumber(0, rows - 1);
    chosenPos.col = rng.getRandomNumber(0, cols - 1);
    const auto chosenTile = fieldData[chosenPos.row][chosenPos.col];
    if (_fieldEmptyDataMarker == chosenTile) {
      break;
    }
  }

  return chosenPos;
}

void Coin::startRespawnAnim(const FieldPos& fieldPos) {
  Point absPos = FieldUtils::getAbsPos(fieldPos);
  absPos += _tileOffset;
  _coinImg.setPosition(absPos);
  _respawnAnim.start();
}

