//Corresponding header
#include "robo_common/layout/entities/robot/animation/PlayerDamageAnimEndCb.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/log/Log.h"

//Own components headers

ErrorCode PlayerDamageAnimEndCb::init(
    const std::function<void()> &onPlayerDamageAnimEndCb) {
  if (nullptr == onPlayerDamageAnimEndCb) {
    LOGERR("Error, nullptr provided for onPlayerDamageAnimEndCb");
    return ErrorCode::FAILURE;
  }
  _onPlayerDamageAnimEndCb = onPlayerDamageAnimEndCb;

  return ErrorCode::SUCCESS;
}

ErrorCode PlayerDamageAnimEndCb::onAnimationEnd() {
  _onPlayerDamageAnimEndCb();
  return ErrorCode::SUCCESS;
}

