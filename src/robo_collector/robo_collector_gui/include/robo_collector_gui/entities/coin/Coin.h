#ifndef ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_
#define ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/animation/FrameAnimation.h"
#include "manager_utils/drawing/animation/PositionAnimation.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/helpers/CollisionObject.h"
#include "robo_collector_gui/field/FieldPos.h"
#include "robo_collector_gui/entities/coin/animation/CoinRespawnAnim.h"
#include "robo_collector_gui/entities/coin/animation/CoinCollectAnimEndCb.h"

//Forward declarations
class CollisionWatcher;

struct CoinConfig {
  //x and y offset from the top-left part of a tile
  Point tileOffset;
  uint64_t rsrcId = 0;
  int32_t rotateAnimTimerId = 0;
  int32_t collectAnimTimerId = 0;
  int32_t respawnAnimTimerId = 0;
  int32_t coinScore = 0;
  char fieldDataMarker = '!';
  char fieldEmptyDataMarker = '?';
  IncrCollectedCoinsCb incrCollectedCoinsCb;
  SetFieldDataMarkerCb setFieldDataMarkerCb;
  ResetFieldDataMarkerCb resetFieldDataMarkerCb;
  GetFieldDataCb getFieldDataCb;
  GetFieldEmptyDataMarkerCb _getFieldEmptyDataMarkerCb;
  CollisionWatcher* collisionWatcher = nullptr;
};

class Coin final : public CollisionObject {
public:
  int32_t init(const CoinConfig& cfg);
  void deinit();
  void draw() const;
  void onAnimEnd(CoinAnimType coinAnimType);

private:
  void startCollectAnim();
  void registerCollision(const Rectangle& intersectRect,
                         CollisionDamageImpact impact) override;
  Rectangle getBoundary() const override;

  FieldPos choseRespawnLocation();
  void startRespawnAnim(const FieldPos& fieldPos);

  Image _coinImg;
  FrameAnimation _rotateAnim;
  PositionAnimation _colllectAnim;
  CoinRespawnAnim _respawnAnim;

  CoinCollectAnimEndCb _coinCollectAnimEndCb;

  Point _tileOffset;
  int32_t _collectAnimTimerId = 0;
  int32_t _coinScore = 0;
  char _fieldDataMarker = '!';
  char _fieldEmptyDataMarker = '?';

  IncrCollectedCoinsCb _incrCollectedCoinsCb;
  SetFieldDataMarkerCb _setFieldDataMarkerCb;
  ResetFieldDataMarkerCb _resetFieldDataMarkerCb;
  GetFieldDataCb _getFieldDataCb;
  GetFieldEmptyDataMarkerCb _getFieldEmptyDataMarkerCb;

  CollisionWatcher* _collisionWatcher = nullptr;
};

#endif /* ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_ */
