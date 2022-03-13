#ifndef ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_
#define ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/helpers/CollisionObject.h"
#include "manager_utils/drawing/animation/FrameAnimation.h"
#include "manager_utils/drawing/animation/PositionAnimation.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/layout/entities/coin/animation/CoinRespawnAnim.h"
#include "robo_collector_gui/layout/entities/coin/animation/CoinCollectAnimEndCb.h"

//Forward declarations
class CollisionWatcher;

struct CoinOutInterface {
  SetFieldDataMarkerCb setFieldDataMarkerCb;
  ResetFieldDataMarkerCb resetFieldDataMarkerCb;
  GetFieldDescriptionCb getFieldDescriptionCb;
  IncrCollectedCoinsCb incrCollectedCoinsCb;
  IsPlayerTurnActiveCb isPlayerTurnActiveCb;
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
  ErrorCode init(const CoinConfig &cfg, const CoinOutInterface &interface);
  void deinit();
  void draw() const;
  void onAnimEnd(CoinAnimType coinAnimType);

private:
  ErrorCode initOutInterface(const CoinOutInterface &interface);
  ErrorCode initRespawnAnim();
  ErrorCode initRotateAnim(const FieldPos &fieldPos);
  void onInitEnd();

  void startCollectAnim();
  void registerCollision(const Rectangle &intersectRect,
                         CollisionDamageImpact impact) override;
  Rectangle getBoundary() const override;

  FieldPos choseRespawnLocation();
  void startRespawnAnim();

  Image _coinImg;
  FrameAnimation _rotateAnim;
  PositionAnimation _colllectAnim;
  CoinRespawnAnim _respawnAnim;

  CoinCollectAnimEndCb _coinCollectAnimEndCb;

  FieldPos _fieldPos;
  CoinConfig _state;
  CoinOutInterface _outInterface;
};

#endif /* ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_ */
