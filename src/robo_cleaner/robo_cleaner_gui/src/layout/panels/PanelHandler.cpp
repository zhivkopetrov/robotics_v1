//Corresponding header
#include "robo_cleaner_gui/layout/panels/PanelHandler.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t PanelHandler::init(const PanelHandlerConfig &cfg,
                           const PanelHandlerOutInterface &interface) {
  //TODO attach gameWonCb on end of triple star animation
  //SINGLE_STAR will be to reveal the whole tile map
  //DOUBLE_STAR will be to clean all the rubbish
  //TRIPLE_STAR will be to successfully execute reveal + clean
  //            and get to the charging station with full health

  const auto achievementWonCb = interface.startAchievementWonAnimCb;
  auto panelPos = Point(1250, 50);
  const auto lightGoldColor = Color(0xD4AF37FF);
  NumberCounterPanelUtilityConfig numberCounterPanelUtilityCfg;
  numberCounterPanelUtilityCfg.targetReachedCb = [achievementWonCb](){
    achievementWonCb(Achievement::SINGLE_STAR);
  };

  numberCounterPanelUtilityCfg.pos = panelPos;
  numberCounterPanelUtilityCfg.textColor = lightGoldColor;
  if (SUCCESS !=
      _tilePanel.init(cfg.tilePanelCfg, numberCounterPanelUtilityCfg)) {
    LOGERR("Error, tilePanelCfg.init() failed");
    return FAILURE;
  }

  panelPos.y += 165;
  numberCounterPanelUtilityCfg.targetReachedCb = [achievementWonCb](){
    achievementWonCb(Achievement::DOUBLE_STAR);
  };
  numberCounterPanelUtilityCfg.pos = panelPos;
  if (SUCCESS !=
      _rubbishPanel.init(cfg.rubbishPanelCfg, numberCounterPanelUtilityCfg)) {
    LOGERR("Error, _rubbishPanel.init() failed");
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

  panelPos.y += 95;
  indicatorUtilityCfg.indicatorDepletedCb = interface.energyDepletedCb;
  indicatorUtilityCfg.pos = panelPos;
  if (SUCCESS != _energyPanel.init(cfg.energyPanelCfg, indicatorUtilityCfg)) {
    LOGERR("Error, _energyPanel.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

void PanelHandler::draw() const {
  _tilePanel.draw();
  _rubbishPanel.draw();
  _healthPanel.draw();
  _energyPanel.draw();
}

void PanelHandler::decreaseHealthIndicator(int32_t delta) {
  _healthPanel.decreaseIndicator(delta);
}

void PanelHandler::decreaseEnergyIndicator(int32_t delta) {
  _energyPanel.decreaseIndicator(delta);
}

