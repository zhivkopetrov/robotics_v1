//Corresponding header
#include "robo_common/entities/robot/animation/PlayerDamageAnimEndCb.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t PlayerDamageAnimEndCb::init(
    const std::function<void()> &onPlayerDamageAnimEndCb) {
  if (nullptr == onPlayerDamageAnimEndCb) {
    LOGERR("Error, nullptr provided for onPlayerDamageAnimEndCb");
    return FAILURE;
  }
  _onPlayerDamageAnimEndCb = onPlayerDamageAnimEndCb;

  return SUCCESS;
}

int32_t PlayerDamageAnimEndCb::onAnimationEnd() {
  _onPlayerDamageAnimEndCb();
  return SUCCESS;
}

