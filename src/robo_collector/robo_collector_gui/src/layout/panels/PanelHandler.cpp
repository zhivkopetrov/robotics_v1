//Corresponding header
#include "robo_collector_gui/layout/panels/PanelHandler.h"

//System headers

//Other libraries headers
#include "utils/log/Log.h"

//Own components headers

ErrorCode PanelHandler::init(const PanelHandlerConfig &cfg,
                             const PanelHandlerOutInterface &interface) {
  if (ErrorCode::SUCCESS != validateInterface(interface)) {
    LOGERR("Error, validateInterface() failed");
    return ErrorCode::FAILURE;
  }

  const auto startGameLostAnimCb = interface.startGameLostAnimCb;
  const auto startGameWonAnimCb = interface.startGameWonAnimCb;
  const auto shutdownControllerCb = interface.shutdownControllerCb;

  auto panelPos = Point(1250, 50);
  TimePanelUtilityConfig timePanelUtilityCfg;
  timePanelUtilityCfg.timeFinishedCb =
      [startGameLostAnimCb, shutdownControllerCb](){
    shutdownControllerCb();
    startGameLostAnimCb();
  };
  timePanelUtilityCfg.pos = panelPos;
  if (ErrorCode::SUCCESS != _timePanel.init(cfg.timePanelCfg,
          timePanelUtilityCfg)) {
    LOGERR("Error, _timePanel.init() failed");
    return ErrorCode::FAILURE;
  }

  panelPos.y += 165;
  const auto lightGoldColor = Color(0xD4AF37FF);
  NumberCounterPanelUtilityConfig numberCounterPanelUtilityCfg;
  numberCounterPanelUtilityCfg.targetReachedCb =
      [startGameWonAnimCb, shutdownControllerCb](){
    shutdownControllerCb();
    startGameWonAnimCb();
  };
  numberCounterPanelUtilityCfg.pos = panelPos;
  numberCounterPanelUtilityCfg.textColor = lightGoldColor;
  if (ErrorCode::SUCCESS != _coinPanel.init(cfg.coinPanelCfg,
          numberCounterPanelUtilityCfg)) {
    LOGERR("Error, _coinPanel.init() failed");
    return ErrorCode::FAILURE;
  }

  panelPos.y += 175;
  IndicatorPanelUtilityConfig indicatorUtilityCfg;
  indicatorUtilityCfg.indicatorDepletedCb =
      [startGameLostAnimCb, shutdownControllerCb](){
    shutdownControllerCb();
    startGameLostAnimCb();
  };
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

void PanelHandler::modifyHealthIndicator(int32_t damage) {
  _healthPanel.modifyIndicator(-1 * damage);
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

  if (nullptr == interface.shutdownControllerCb) {
    LOGERR("Error, nullptr provided for ShutdownControllerCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

