#ifndef ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_
#define ROBO_COLLECTOR_GUI_ENTITIES_COIN_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/animation/FrameAnimation.h"

//Own components headers

//Forward declarations

struct CoinConfig {
  uint64_t rsrcId = 0;
  Point pos;
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
