//Corresponding header
#include "robo_collector_gui/entities/coin/CoinHandler.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t CoinHandler::init(const CoinHandlerConfig &cfg) {
  const int32_t rsrcIdsSize = static_cast<int32_t>(cfg.animRsrcIds.size());
  if (cfg.maxCoins != rsrcIdsSize) {
    LOGERR(
        "Error, coinAnimRsrcIds.size() is: %d, while it should be exactly: %d",
        rsrcIdsSize, cfg.maxCoins);
    return FAILURE;
  }
  _coins.resize(cfg.maxCoins);

  const int32_t fieldDataMarkersSize =
      static_cast<int32_t>(cfg.fieldDataMarkers.size());
  if (cfg.maxCoins != fieldDataMarkersSize) {
    LOGERR(
        "Error, fieldDataMarkers.size() is: %d, while it should be exactly: %d",
        fieldDataMarkersSize, cfg.maxCoins);
    return FAILURE;
  }

  constexpr auto goldCoinScore = 3;
  constexpr auto coinOffsetFromTile = 30;
  CoinConfig coinCfg;
  coinCfg.collisionWatcher = cfg.collisionWatcher;
  coinCfg.incrCollectedCoinsCb = cfg.incrCollectedCoinsCb;
  coinCfg.setFieldDataMarkerCb = cfg.setFieldDataMarkerCb;
  coinCfg.resetFieldDataMarkerCb = cfg.resetFieldDataMarkerCb;
  coinCfg.getFieldDataCb = cfg.getFieldDataCb;
  coinCfg.fieldEmptyDataMarker = cfg.fieldEmptyDataMarker;
  coinCfg.tileOffset = Point(coinOffsetFromTile, coinOffsetFromTile);

  for (int32_t i = 0; i < cfg.maxCoins; ++i) {
    coinCfg.coinScore = goldCoinScore - i;
    coinCfg.rsrcId = cfg.animRsrcIds[i];
    coinCfg.fieldDataMarker = cfg.fieldDataMarkers[i];
    coinCfg.rotateAnimTimerId = cfg.rotateAnimFirstTimerId + i;
    coinCfg.collectAnimTimerId = cfg.collectAnimFirstTimerId + i;
    coinCfg.respawnAnimTimerId = cfg.respawnAnimFirstTimerId + i;
    if (SUCCESS != _coins[i].init(coinCfg)) {
      LOGERR("Error in _coins[%d].init()", i);
      return FAILURE;
    }
  }

  return SUCCESS;
}

void CoinHandler::deinit() {
  for (auto &coin : _coins) {
    coin.deinit();
  }
}

void CoinHandler::draw() const {
  for (const auto &coin : _coins) {
    coin.draw();
  }
}


