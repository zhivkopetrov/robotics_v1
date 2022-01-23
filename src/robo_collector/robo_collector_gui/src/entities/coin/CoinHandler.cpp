//Corresponding header
#include "robo_collector_gui/entities/coin/CoinHandler.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/rng/Rng.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t CoinHandler::init(const CoinHandlerConfig &cfg) {
  if (nullptr == cfg.setFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for RobotCfg setFieldDataMarkerCb");
    return FAILURE;
  }
  _setFieldDataMarkerCb = cfg.setFieldDataMarkerCb;

  if (nullptr == cfg.resetFieldDataMarkerCb) {
    LOGERR("Error, nullptr provided for RobotCfg resetFieldDataMarkerCb");
    return FAILURE;
  }
  _resetFieldDataMarkerCb = cfg.resetFieldDataMarkerCb;

  if (nullptr == cfg.getFieldDataCb) {
    LOGERR("Error, nullptr provided for RobotCfg getFieldDataCb");
    return FAILURE;
  }
  _getFieldDataCb = cfg.getFieldDataCb;

  const int32_t rsrcIdsSize = static_cast<int32_t>(cfg.animRsrcIds.size());
  if (cfg.maxCoins != rsrcIdsSize) {
    LOGERR(
        "Error, coinAnimRsrcIds.size() is: %d, while it should be exactly: %d",
        rsrcIdsSize, cfg.maxCoins);
    return FAILURE;
  }
  _coins.resize(cfg.maxCoins);

  constexpr auto coinOffsetFromTile = 30;
  CoinConfig coinCfg;
  coinCfg.fieldPos.row = 4;
  coinCfg.tileOffset = Point(coinOffsetFromTile, coinOffsetFromTile);

  for (int32_t i = 0; i < cfg.maxCoins; ++i) {
    coinCfg.rsrcId = cfg.animRsrcIds[i];
    coinCfg.rotateAnimTimerId = cfg.rotateAnimFirstTimerId + i;
    coinCfg.collectAnimTimerId = cfg.collectAnimFirstTimerId + i;
    coinCfg.fieldPos.col = i;
    if (SUCCESS != _coins[i].init(coinCfg)) {
      LOGERR("Error in _coins[%d].init()", i);
      return FAILURE;
    }
  }

  return SUCCESS;
}

void CoinHandler::draw() const {
  for (const auto &coin : _coins) {
    coin.draw();
  }
}

void CoinHandler::handleEvent(const InputEvent& e) {
  if (TouchEvent::KEYBOARD_RELEASE != e.type) {
    return;
  }

  if (Keyboard::KEY_UP == e.key) {
    _coins[0].startCollectAnim();
  } else if (Keyboard::KEY_RIGHT == e.key) {
    _coins[1].startCollectAnim();
  } else if (Keyboard::KEY_LEFT == e.key) {
    _coins[2].startCollectAnim();
  }
}


