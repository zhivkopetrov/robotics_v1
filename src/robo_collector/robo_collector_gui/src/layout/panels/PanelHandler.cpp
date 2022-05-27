//Corresponding header
#include "robo_collector_gui/layout/panels/PanelHandler.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers

ErrorCode PanelHandler::init(const PanelHandlerConfig &cfg,
                             const PanelHandlerOutInterface &interface) {
  //TODO attach gameWonCb on end of triple star animation
  //SINGLE_STAR will be don't collect silver coins
  //DOUBLE_STAR will be to collect at least 3 bronze coins
  //TRIPLE_STAR will be to finish the game
  //            with at least 30 seconds left on the timer

  if (ErrorCode::SUCCESS != validateInterface(interface)) {
    LOGERR("Error, validateInterface() failed");
    return ErrorCode::FAILURE;
  }

  auto panelPos = Point(1250, 50);
  TimePanelUtilityConfig timePanelUtilityCfg;
  timePanelUtilityCfg.timeFinishedCb = interface.startGameLostAnimCb;
  timePanelUtilityCfg.pos = panelPos;
  if (ErrorCode::SUCCESS != _timePanel.init(cfg.timePanelCfg,
          timePanelUtilityCfg)) {
    LOGERR("Error, _timePanel.init() failed");
    return ErrorCode::FAILURE;
  }

  panelPos.y += 165;
  const auto lightGoldColor = Color(0xD4AF37FF);
  NumberCounterPanelUtilityConfig numberCounterPanelUtilityCfg;
  numberCounterPanelUtilityCfg.targetReachedCb = interface.startGameWonAnimCb;
  numberCounterPanelUtilityCfg.pos = panelPos;
  numberCounterPanelUtilityCfg.textColor = lightGoldColor;
  if (ErrorCode::SUCCESS != _coinPanel.init(cfg.coinPanelCfg,
          numberCounterPanelUtilityCfg)) {
    LOGERR("Error, _coinPanel.init() failed");
    return ErrorCode::FAILURE;
  }

  panelPos.y += 175;
  IndicatorPanelUtilityConfig indicatorUtilityCfg;
  indicatorUtilityCfg.indicatorDepletedCb = interface.startGameLostAnimCb;
  indicatorUtilityCfg.pos = panelPos;
  if (ErrorCode::SUCCESS != _healthPanel.init(cfg.healthPanelCfg,
          indicatorUtilityCfg)) {
    LOGERR("Error, _healthPanel.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
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

ErrorCode PanelHandler::validateInterface(
    const PanelHandlerOutInterface &interface) const {
  if (nullptr == interface.startGameLostAnimCb) {
    LOGERR("Error, nullptr provided for StartGameLostAnimCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == interface.startGameWonAnimCb) {
    LOGERR("Error, nullptr provided for StartGameWonAnimCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

