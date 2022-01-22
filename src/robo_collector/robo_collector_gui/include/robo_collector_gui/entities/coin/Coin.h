#ifndef ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_
#define ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/animation/FrameAnimation.h"
#include "manager_utils/drawing/animation/PositionAnimation.h"

//Own components headers
#include "robo_collector_gui/field/FieldPos.h"

//Forward declarations

struct CoinConfig {
  FieldPos fieldPos;
  //x and y offset from the top-left part of a tile
  Point tileOffset;
  uint64_t rsrcId = 0;
  int32_t rotateAnimTimerId = 0;
  int32_t collectAnimTimerId = 0;
};

class Coin {
public:
  int32_t init(const CoinConfig& cfg);

  void draw() const;

  void startCollectAnim();

private:
  Image _coinImg;
  FrameAnimation _rotateAnim;
  PositionAnimation _posAnim;

  int32_t collectAnimTimerId = 0;
};

#endif /* ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_ */
