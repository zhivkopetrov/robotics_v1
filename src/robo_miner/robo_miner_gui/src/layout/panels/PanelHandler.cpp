//Corresponding header
#include "robo_miner_gui/layout/panels/PanelHandler.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t PanelHandler::init(const PanelHandlerConfig &cfg,
                           const PanelHandlerOutInterface &interface) {
  //TODO attach gameWonCb on end of triple star animation
  //the DOUBLE_STAR will be the longest sequence algorithm completion
  auto panelPos = Point(1250, 50);
  const auto lightGoldColor = Color(0xD4AF37FF);
  NumberCounterPanelUtilityConfig numberCounterPanelUtilityCfg;
  numberCounterPanelUtilityCfg.targetReachedCb = interface.fieldMapRevelealedCb;

  numberCounterPanelUtilityCfg.pos = panelPos;
  numberCounterPanelUtilityCfg.textColor = lightGoldColor;
  if (SUCCESS !=
      _tilePanel.init(cfg.tilePanelCfg, numberCounterPanelUtilityCfg)) {
    LOGERR("Error, tilePanelCfg.init() failed");
    return FAILURE;
  }

  const auto achievementWonCb = interface.startAchievementWonAnimCb;
  panelPos.y += 165;
  numberCounterPanelUtilityCfg.targetReachedCb = [achievementWonCb](){
    achievementWonCb(Achievement::TRIPLE_STAR);
  };
  numberCounterPanelUtilityCfg.pos = panelPos;
  if (SUCCESS !=
      _crystalPanel.init(cfg.crystalPanelCfg, numberCounterPanelUtilityCfg)) {
    LOGERR("Error, _crystalPanel.init() failed");
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
  _tilePanel.draw();
  _crystalPanel.draw();
  _healthPanel.draw();
}

void PanelHandler::decreaseHealthIndicator(int32_t damage) {
  _healthPanel.decreaseIndicator(damage);
}

void PanelHandler::onTileRevealed() {
  _tilePanel.increaseCounter(1);
}

void PanelHandler::onCrystalMined() {
  _crystalPanel.increaseCounter(1);
}

