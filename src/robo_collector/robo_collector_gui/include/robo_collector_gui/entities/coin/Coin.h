#ifndef ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_
#define ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/animation/FrameAnimation.h"

//Own components headers
#include "robo_collector_gui/field/FieldPos.h"

//Forward declarations

struct CoinConfig {
  FieldPos fieldPos;
  Point tileOffset;
  uint64_t rsrcId = 0;
  //x and y offset from the top-left part of a tile
  int32_t timerId = 0;
};

class Coin {
public:
  int32_t init(const CoinConfig& cfg);

  void draw() const;

private:
  FrameAnimation _anim;
};

#endif /* ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_ */
