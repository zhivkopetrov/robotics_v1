#ifndef ROBO_COLLECTOR_GUI_PLAYERDAMAGEANIMENDCB_H_
#define ROBO_COLLECTOR_GUI_PLAYERDAMAGEANIMENDCB_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <functional>

//Other libraries headers
#include "manager_utils/drawing/animation/AnimationEndCb.h"

//Own components headers

//Forward declarations

class PlayerDamageAnimEndCb final : public AnimationEndCb {
public:
  int32_t init(const std::function<void()>& onPlayerDamageAnimEndCb);

  int32_t onAnimationEnd() override;

private:
  std::function<void()> _onPlayerDamageAnimEndCb;

private:
};

#endif /* ROBO_COLLECTOR_GUI_PLAYERDAMAGEANIMENDCB_H_ */
