//Corresponding header
#include "robo_collector_gui/entities/coin/CoinAnimEndCb.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t CoinAnimEndCb::init(
    const std::function<void()> &notifyCoinOnAnimEndCb) {
  if (nullptr == notifyCoinOnAnimEndCb) {
    LOGERR("Error, nullptr provided for notifyCoinOnAnimEndCb");
    return FAILURE;
  }
  _notifyCoinOnAnimEndCb = notifyCoinOnAnimEndCb;

  return SUCCESS;
}

int32_t CoinAnimEndCb::onAnimationEnd() {
  _notifyCoinOnAnimEndCb();
  return SUCCESS;
}
