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

struct CoinOutInterface {
  SetFieldDataMarkerCb setFieldDataMarkerCb;
  ResetFieldDataMarkerCb resetFieldDataMarkerCb;
  GetFieldDataCb getFieldDataCb;
  IncrCollectedCoinsCb incrCollectedCoinsCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

struct CoinConfig {
  //x and y offset from the top-left part of a tile
  Point tileOffset;
  uint64_t rsrcId = 0;
  int32_t rotateAnimTimerId = 0;
  int32_t collectAnimTimerId = 0;
  int32_t respawnAnimTimerId = 0;
  int32_t coinScore = 0;
  char fieldMarker = '!';
  char fieldEmptyMarker = '?';
};

class Coin final : public CollisionObject {
public:
  int32_t init(const CoinConfig &cfg, const CoinOutInterface &interface);
  void deinit();
  void draw() const;
  void onAnimEnd(CoinAnimType coinAnimType);

private:
  int32_t initOutInterface(const CoinOutInterface &interface);
  int32_t initRespawnAnim();
  int32_t initRotateAnim(const FieldPos &fieldPos);
  void onInitEnd();

  void startCollectAnim();
  void registerCollision(const Rectangle &intersectRect,
                         CollisionDamageImpact impact) override;
  Rectangle getBoundary() const override;

  FieldPos choseRespawnLocation();
  void startRespawnAnim(const FieldPos &fieldPos);

  Image _coinImg;
  FrameAnimation _rotateAnim;
  PositionAnimation _colllectAnim;
  CoinRespawnAnim _respawnAnim;

  CoinCollectAnimEndCb _coinCollectAnimEndCb;

  FieldPos _fieldPos;
  CoinConfig _cfg;
  CoinOutInterface _outInterface;
};

#endif /* ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_ */
