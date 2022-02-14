//Corresponding header
#include "robo_collector_gui/entities/coin/Coin.h"

//C system headers

//C++ system headers
#include <cmath>

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "robo_common/helpers/CollisionWatcher.h"
#include "utils/rng/Rng.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto TARGET_COLLECT_ANIM_X = 1285;
constexpr auto TARGET_COLLECT_ANIM_Y = 243;
}

int32_t Coin::init(const CoinConfig &cfg, const CoinOutInterface& interface) {
  _state = cfg;

  if (SUCCESS != initOutInterface(interface)) {
    LOGERR("Error, initOutInterface() failed");
    return FAILURE;
  }
  _coinImg.create(_state.rsrcId);
  _fieldPos = choseRespawnLocation();

  if (SUCCESS != initRotateAnim(_fieldPos)) {
    LOGERR("Error, initRotateAnim() failed");
    return FAILURE;
  }

  if (SUCCESS != initRespawnAnim()) {
    LOGERR("Error, initRespawnAnim() failed");
    return FAILURE;
  }

  onInitEnd();
  return SUCCESS;
}

void Coin::deinit() {
  if (_outInterface.collisionWatcher) {
    _outInterface.collisionWatcher->unregisterObject(_collisionObjHandle);
  }
}

void Coin::draw() const {
  _coinImg.draw();
}

void Coin::onAnimEnd(CoinAnimType coinAnimType) {
  if (CoinAnimType::COLLECT == coinAnimType) {
    _outInterface.incrCollectedCoinsCb(_state.coinScore);
    startRespawnAnim();
  } else if(CoinAnimType::RESPAWN == coinAnimType) {
    _outInterface.setFieldDataMarkerCb(
        FieldUtils::getFieldPos(_coinImg.getPosition()), _state.fieldMarker);
  } else {
    LOGERR("Logical error, received wrong anim type: %d",
        getEnumValue(coinAnimType));
  }
}

int32_t Coin::initRespawnAnim() {
  const auto animEndCb =
      std::bind(&Coin::onAnimEnd, this, std::placeholders::_1);
  if (SUCCESS != _coinCollectAnimEndCb.init(animEndCb)) {
    LOGERR("Error, coinAnimEndCb.init() failed");
    return FAILURE;
  }

  CoinRespawnAnimConfig coinRespawnAnimCfg;
  coinRespawnAnimCfg.coinImg = &_coinImg;
  coinRespawnAnimCfg.timerId = _state.respawnAnimTimerId;
  coinRespawnAnimCfg.animEndCb = animEndCb;
  if (SUCCESS != _respawnAnim.init(coinRespawnAnimCfg)) {
    LOGERR("Error, _respawnAnim.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t Coin::initRotateAnim(const FieldPos &fieldPos) {
  AnimBaseConfig animCfg;
  animCfg.timerId = _state.rotateAnimTimerId;
  animCfg.timerInterval = 75;
  animCfg.startPos = FieldUtils::getAbsPos(fieldPos);
  animCfg.startPos += _state.tileOffset;
  animCfg.animDirection = AnimDir::FORWARD;
  animCfg.animImageType = AnimImageType::EXTERNAL;
  animCfg.externalImage = &_coinImg;

  if (SUCCESS != _rotateAnim.configure(animCfg)) {
    LOGERR("Coin rotateAnim configure failed");
    return FAILURE;
  }

  return SUCCESS;
}

void Coin::onInitEnd() {
  _outInterface.setFieldDataMarkerCb(_fieldPos, _state.fieldMarker);

  _collisionObjHandle = _outInterface.collisionWatcher->registerObject(
      this, CollisionDamageImpact::NO);

  _rotateAnim.start();
}

int32_t Coin::initOutInterface(const CoinOutInterface &interface) {
  _outInterface = interface;

  if (nullptr == _outInterface.collisionWatcher) {
    LOGERR("Error, nullptr provided for collisionWatcher");
    return FAILURE;
  }

  if (nullptr == _outInterface.incrCollectedCoinsCb) {
    LOGERR("Error, nullptr provided for incrCollectedCoinsCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.setFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for setFieldDataMarkerCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.resetFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for resetFieldDataMarkerCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.getFieldDataCb) {
    LOGERR("Error, nullptr provided for getFieldDataCb");
    return FAILURE;
  }

  if (nullptr == _outInterface.isPlayerTurnActiveCb) {
    LOGERR("Error, nullptr provided for IsPlayerTurnActiveCb");
    return FAILURE;
  }

  return SUCCESS;
}

void Coin::startCollectAnim() {
  AnimBaseConfig cfg;
  cfg.timerId = _state.collectAnimTimerId;
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

  if (SUCCESS != _colllectAnim.configure(cfg, endPos, numberOfSteps,
          &_coinCollectAnimEndCb, PosAnimType::ONE_DIRECTIONAL)) {
    LOGERR("Error, _posAnim.configure() failed");
  }
  _colllectAnim.start();
}

void Coin::registerCollision([[maybe_unused]]const Rectangle &intersectRect,
                             [[maybe_unused]]CollisionDamageImpact impact) {
  const bool isCollisionFromPlayer = _outInterface.isPlayerTurnActiveCb();
  if (isCollisionFromPlayer) {
    startCollectAnim();
  } else {
    startRespawnAnim();
  }
}

Rectangle Coin::getBoundary() const {
  return _coinImg.getImageRect();
}

FieldPos Coin::choseRespawnLocation() {
  const auto& fieldData = _outInterface.getFieldDataCb();
  const auto lastRowIdx = fieldData.size() - 1;
  const auto lastColIdx = fieldData[0].size() - 1;
  auto& rng = Rng::getInstance();
  FieldPos chosenPos;

  while (true) {
    chosenPos.row = rng.getRandomNumber(0, lastRowIdx);
    chosenPos.col = rng.getRandomNumber(0, lastColIdx);
    const auto chosenTile = fieldData[chosenPos.row][chosenPos.col];
    if (_state.fieldEmptyMarker == chosenTile) {
      break;
    }
  }

  return chosenPos;
}

void Coin::startRespawnAnim() {
  _outInterface.resetFieldDataMarkerCb(
      FieldUtils::getFieldPos(_coinImg.getPosition()));
  const auto fieldPos = choseRespawnLocation();

  Point absPos = FieldUtils::getAbsPos(fieldPos);
  absPos += _state.tileOffset;
  _coinImg.setPosition(absPos);
  _respawnAnim.start();
}

