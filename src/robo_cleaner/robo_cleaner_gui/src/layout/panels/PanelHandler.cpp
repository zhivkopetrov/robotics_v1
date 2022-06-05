//Corresponding header
#include "robo_cleaner_gui/layout/panels/PanelHandler.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

ErrorCode PanelHandler::init(const PanelHandlerConfig &cfg,
                             const PanelHandlerOutInterface &interface) {
  if (ErrorCode::SUCCESS != validateInterface(interface)) {
    LOGERR("Error, validateInterface() failed");
    return ErrorCode::FAILURE;
  }

  //TODO attach gameWonCb on end of triple star animation
  //SINGLE_STAR will be to reveal the whole tile map
  //DOUBLE_STAR will be to clean all the rubbish
  //TRIPLE_STAR will be to successfully execute reveal + clean
  //            and get to the charging station with full health

  const auto achievementWonCb = interface.startAchievementWonAnimCb;
  const auto fieldMapRevelealedCb = interface.fieldMapRevelealedCb;
  auto panelPos = Point(1250, 50);
  const auto lightGoldColor = Color(0xD4AF37FF);
  NumberCounterPanelUtilityConfig numberCounterPanelUtilityCfg;
  numberCounterPanelUtilityCfg.targetReachedCb = [achievementWonCb,
                                                  fieldMapRevelealedCb]() {
    fieldMapRevelealedCb();
    achievementWonCb(Achievement::SINGLE_STAR);
  };

  numberCounterPanelUtilityCfg.pos = panelPos;
  numberCounterPanelUtilityCfg.textColor = lightGoldColor;
  if (ErrorCode::SUCCESS != _tilePanel.init(cfg.tilePanelCfg,
          numberCounterPanelUtilityCfg)) {
    LOGERR("Error, tilePanelCfg.init() failed");
    return ErrorCode::FAILURE;
  }

  const auto fieldMapCleanedCb = interface.fieldMapCleanedCb;
  panelPos.y += 165;
  numberCounterPanelUtilityCfg.targetReachedCb = [achievementWonCb,
                                                  fieldMapCleanedCb]() {
    fieldMapCleanedCb();
    achievementWonCb(Achievement::DOUBLE_STAR);
  };
  numberCounterPanelUtilityCfg.pos = panelPos;
  if (ErrorCode::SUCCESS != _rubbishPanel.init(cfg.rubbishPanelCfg,
          numberCounterPanelUtilityCfg)) {
    LOGERR("Error, _rubbishPanel.init() failed");
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

  panelPos.y += 95;
  indicatorUtilityCfg.indicatorDepletedCb = interface.energyDepletedCb;
  indicatorUtilityCfg.pos = panelPos;
  if (ErrorCode::SUCCESS != _energyPanel.init(cfg.energyPanelCfg,
          indicatorUtilityCfg)) {
    LOGERR("Error, _energyPanel.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
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

void PanelHandler::onTileRevealed() {
  _tilePanel.increaseCounter(1);
}

void PanelHandler::onTileCleaned() {
  _rubbishPanel.increaseCounter(1);
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

  if (nullptr == interface.fieldMapRevelealedCb) {
    LOGERR("Error, nullptr provided for FieldMapRevelealedCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == interface.fieldMapCleanedCb) {
    LOGERR("Error, nullptr provided for FieldMapCleanedCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

