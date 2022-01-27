//Corresponding header
#include "robo_collector_gui/entities/coin/CoinHandler.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t CoinHandler::init(const CoinHandlerConfig &cfg,
                          const CoinOutInterface& interface) {
  const int32_t rsrcIdsSize = static_cast<int32_t>(cfg.animRsrcIds.size());
  if (cfg.maxCoins != rsrcIdsSize) {
    LOGERR(
        "Error, coinAnimRsrcIds.size() is: %d, while it should be exactly: %d",
        rsrcIdsSize, cfg.maxCoins);
    return FAILURE;
  }
  _coins.resize(cfg.maxCoins);

  const int32_t fieldMarkersSize =
      static_cast<int32_t>(cfg.fieldMarkers.size());
  if (cfg.maxCoins != fieldMarkersSize) {
    LOGERR(
        "Error, fieldMarkers.size() is: %d, while it should be exactly: %d",
        fieldMarkersSize, cfg.maxCoins);
    return FAILURE;
  }

  constexpr auto goldCoinScore = 3;
  constexpr auto coinOffsetFromTile = 30;
  CoinConfig coinCfg;
  coinCfg.tileOffset = Point(coinOffsetFromTile, coinOffsetFromTile);

  for (int32_t i = 0; i < cfg.maxCoins; ++i) {
    coinCfg.fieldEmptyMarker = cfg.fieldEmptyMarker;
    coinCfg.coinScore = goldCoinScore - i;
    coinCfg.rsrcId = cfg.animRsrcIds[i];
    coinCfg.fieldMarker = cfg.fieldMarkers[i];
    coinCfg.rotateAnimTimerId = cfg.rotateAnimFirstTimerId + i;
    coinCfg.collectAnimTimerId = cfg.collectAnimFirstTimerId + i;
    coinCfg.respawnAnimTimerId = cfg.respawnAnimFirstTimerId + i;
    if (SUCCESS != _coins[i].init(coinCfg, interface)) {
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


