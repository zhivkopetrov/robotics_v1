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
  //TODO attach gameWonCb on end of triple star animation
  //SINGLE_STAR will be don't collect silver coins
  //DOUBLE_STAR will be to collect at least 3 bronze coins
  //TRIPLE_STAR will be to finish the game
  //            with at least 1 minute left on the timer

  auto panelPos = Point(1250, 50);
  TimePanelUtilityConfig timePanelUtilityCfg;
  timePanelUtilityCfg.timeFinishedCb = interface.startGameLostAnimCb;
  timePanelUtilityCfg.pos = panelPos;
  if (SUCCESS != _timePanel.init(cfg.timePanelCfg, timePanelUtilityCfg)) {
    LOGERR("Error, _timePanel.init() failed");
    return FAILURE;
  }

  panelPos.y += 165;
  const auto lightGoldColor = Color(0xD4AF37FF);
  NumberCounterPanelUtilityConfig numberCounterPanelUtilityCfg;
  numberCounterPanelUtilityCfg.targetReachedCb = interface.startGameWonAnimCb;
  numberCounterPanelUtilityCfg.pos = panelPos;
  numberCounterPanelUtilityCfg.textColor = lightGoldColor;
  if (SUCCESS !=
      _coinPanel.init(cfg.coinPanelCfg, numberCounterPanelUtilityCfg)) {
    LOGERR("Error, _coinPanel.init() failed");
    return FAILURE;
  }

  panelPos.y += 175;
  IndicatorPanelUtilityConfig indicatorUtilityCfg;
  indicatorUtilityCfg.indicatorDepletedCb = interface.startGameLostAnimCb;
  indicatorUtilityCfg.pos = panelPos;
  if (SUCCESS != _healthPanel.init(cfg.healthPanelCfg, indicatorUtilityCfg)) {
    LOGERR("Error, _healthPanel.init() failed");
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
  _coinPanel.increaseCounter(coins);
}

