#ifndef ROBO_COLLECTOR_GUI_COINCOLLECTANIMENDCB_H_
#define ROBO_COLLECTOR_GUI_COINCOLLECTANIMENDCB_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <functional>

//Other libraries headers
#include "manager_utils/drawing/animation/AnimationEndCb.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"

//Forward declarations

class CoinCollectAnimEndCb final : public AnimationEndCb {
public:
  int32_t init(const std::function<void(CoinAnimType)>& coinOnAnimEndCb);

  int32_t onAnimationEnd() override;

private:
  std::function<void(CoinAnimType)> _coinOnAnimEndCb;
};

#endif /* ROBO_COLLECTOR_GUI_COINCOLLECTANIMENDCB_H_ */
