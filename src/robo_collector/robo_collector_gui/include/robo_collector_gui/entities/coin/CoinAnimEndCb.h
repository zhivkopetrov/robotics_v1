#ifndef ROBO_COLLECTOR_GUI_COINANIMENDCB_H_
#define ROBO_COLLECTOR_GUI_COINANIMENDCB_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <functional>

//Other libraries headers
#include "manager_utils/drawing/animation/AnimationEndCb.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"
#include "robo_collector_gui/field/FieldPos.h"

//Forward declarations

class CoinAnimEndCb final : public AnimationEndCb {
public:
  int32_t init(const std::function<void()>& notifyCoinOnAnimEndCb);

  int32_t onAnimationEnd() override;

private:
  std::function<void()> _notifyCoinOnAnimEndCb;
};

#endif /* ROBO_COLLECTOR_GUI_COINANIMENDCB_H_ */
