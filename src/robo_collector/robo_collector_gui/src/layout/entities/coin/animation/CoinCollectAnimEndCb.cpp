//Corresponding header
#include "robo_collector_gui/layout/entities/coin/animation/CoinCollectAnimEndCb.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t CoinCollectAnimEndCb::init(
    const std::function<void(CoinAnimType)> &coinOnAnimEndCb) {
  if (nullptr == coinOnAnimEndCb) {
    LOGERR("Error, nullptr provided for coinOnAnimEndCb");
    return FAILURE;
  }
  _coinOnAnimEndCb = coinOnAnimEndCb;

  return SUCCESS;
}

int32_t CoinCollectAnimEndCb::onAnimationEnd() {
  _coinOnAnimEndCb(CoinAnimType::COLLECT);
  return SUCCESS;
}
