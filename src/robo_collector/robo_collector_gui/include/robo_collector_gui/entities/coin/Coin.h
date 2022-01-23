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
#include "robo_collector_gui/entities/coin/CoinAnimEndCb.h"

//Forward declarations
class CollisionWatcher;

struct CoinConfig {
  FieldPos fieldPos;
  //x and y offset from the top-left part of a tile
  Point tileOffset;
  uint64_t rsrcId = 0;
  int32_t rotateAnimTimerId = 0;
  int32_t collectAnimTimerId = 0;
  int32_t coinScore = 0;
  IncrCollectedCoinsCb incrCollectedCoinsCb;
  CollisionWatcher* collisionWatcher = nullptr;
};

class Coin final : public CollisionObject {
public:
  int32_t init(const CoinConfig& cfg);
  void deinit();
  void draw() const;
  void onCollectAnimEnd();

private:
  void startCollectAnim();
  void registerCollision(const Rectangle& intersectRect) override;
  Rectangle getBoundary() const override;

  Image _coinImg;
  FrameAnimation _rotateAnim;
  PositionAnimation _posAnim;

  CoinAnimEndCb _coinAnimEndCb;

  int32_t _collectAnimTimerId = 0;
  int32_t _coinScore = 0;

  IncrCollectedCoinsCb _incrCollectedCoinsCb;

  CollisionWatcher* _collisionWatcher = nullptr;
};

#endif /* ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_ */
