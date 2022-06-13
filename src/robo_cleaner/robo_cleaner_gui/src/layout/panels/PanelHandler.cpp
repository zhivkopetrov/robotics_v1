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

  const auto startGameLostAnimCb = interface.startGameLostAnimCb;
  const auto shutdownControllerCb = interface.shutdownControllerCb;
  const auto startAchievementWonCb = interface.startAchievementWonAnimCb;
  const auto fieldMapRevelealedCb = interface.fieldMapRevelealedCb;

  auto panelPos = Point(1250, 50);
  const auto lightGoldColor = Color(0xD4AF37FF);
  NumberCounterPanelUtilityConfig numberCounterPanelUtilityCfg;
  numberCounterPanelUtilityCfg.targetReachedCb = [startAchievementWonCb,
                                                  fieldMapRevelealedCb]() {
    fieldMapRevelealedCb();
    startAchievementWonCb(Achievement::SINGLE_STAR);
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
  numberCounterPanelUtilityCfg.targetReachedCb = [startAchievementWonCb,
                                                  fieldMapCleanedCb]() {
    fieldMapCleanedCb();
    startAchievementWonCb(Achievement::DOUBLE_STAR);
  };
  numberCounterPanelUtilityCfg.pos = panelPos;
  if (ErrorCode::SUCCESS != _rubbishPanel.init(cfg.rubbishPanelCfg,
          numberCounterPanelUtilityCfg)) {
    LOGERR("Error, _rubbishPanel.init() failed");
    return ErrorCode::FAILURE;
  }

  panelPos.y += 175;
  IndicatorPanelUtilityConfig indicatorUtilityCfg;
  indicatorUtilityCfg.indicatorDepletedCb = [startGameLostAnimCb,
                                             shutdownControllerCb]() {
    shutdownControllerCb();
    startGameLostAnimCb();
  };
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

void PanelHandler::modifyHealthIndicator(int32_t delta) {
  _healthPanel.modifyIndicator(-1 * delta);
}

void PanelHandler::modifyEnergyIndicator(int32_t delta) {
  _energyPanel.modifyIndicator(delta);
}

void PanelHandler::onTileRevealed() {
  _tilePanel.increaseCounter(1);
}

void PanelHandler::onTileCleaned() {
  _rubbishPanel.increaseCounter(1);
}

int32_t PanelHandler::getHealthIndicatorValue() const {
  return _healthPanel.getIndicatorValue();
}

ErrorCode PanelHandler::validateInterface(
    const PanelHandlerOutInterface &interface) const {
  if (nullptr == interface.startGameLostAnimCb) {
    LOGERR("Error, nullptr provided for StartGameLostAnimCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == interface.startAchievementWonAnimCb) {
    LOGERR("Error, nullptr provided for startAchievementWonAnimCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == interface.shutdownControllerCb) {
    LOGERR("Error, nullptr provided for ShutdownControllerCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == interface.energyDepletedCb) {
    LOGERR("Error, nullptr provided for EnergyDepletedCb");
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

