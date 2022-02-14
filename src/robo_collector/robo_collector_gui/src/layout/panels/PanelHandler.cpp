//Corresponding header
#include "robo_collector_gui/layout/panels/PanelHandler.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t PanelHandler::init(const PanelHandlerConfig &cfg,
                           const PanelHandlerOutInterface &interface) {
  if (SUCCESS != _healthPanel.init(cfg.healthPanelCfg, interface.gameLostCb)) {
    LOGERR("Error, _healthPanel.init() failed");
    return FAILURE;
  }

  if (SUCCESS != _timePanel.init(cfg.timePanelCfg, interface.gameLostCb)) {
    LOGERR("Error, _timePanel.init() failed");
    return FAILURE;
  }

  if (SUCCESS != _coinPanel.init(cfg.coinPanelCfg, interface.gameWonCb)) {
    LOGERR("Error, _coinPanel.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

void PanelHandler::draw() const {
  _timePanel.draw();
  _healthPanel.draw();
  _coinPanel.draw();
}

void PanelHandler::decreaseHealthIndicator(int32_t damage) {
  _healthPanel.decreaseIndicator(damage);
}

void PanelHandler::increaseCollectedCoins(int32_t coins) {
  _coinPanel.increaseCollectedCoins(coins);
}

