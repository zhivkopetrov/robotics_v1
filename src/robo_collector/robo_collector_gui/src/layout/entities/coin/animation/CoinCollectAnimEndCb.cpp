//Corresponding header
#include "robo_collector_gui/layout/entities/coin/animation/CoinCollectAnimEndCb.h"

//System headers

//Other libraries headers
#include "utils/log/Log.h"

//Own components headers

ErrorCode CoinCollectAnimEndCb::init(
    const std::function<void(CoinAnimType)> &coinOnAnimEndCb) {
  if (nullptr == coinOnAnimEndCb) {
    LOGERR("Error, nullptr provided for coinOnAnimEndCb");
    return ErrorCode::FAILURE;
  }
  _coinOnAnimEndCb = coinOnAnimEndCb;

  return ErrorCode::SUCCESS;
}

ErrorCode CoinCollectAnimEndCb::onAnimationEnd() {
  _coinOnAnimEndCb(CoinAnimType::COLLECT);
  return ErrorCode::SUCCESS;
}
