//Corresponding header
#include "robo_miner_gui/layout/panels/PanelHandler.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
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
  const auto startAchievementWonCb = interface.startAchievementWonAnimCb;

  auto panelPos = Point(1250, 50);
  const auto lightGoldColor = Color(0xD4AF37FF);
  NumberCounterPanelUtilityConfig numberCounterPanelUtilityCfg;
  numberCounterPanelUtilityCfg.targetReachedCb = interface.fieldMapRevelealedCb;

  numberCounterPanelUtilityCfg.pos = panelPos;
  numberCounterPanelUtilityCfg.textColor = lightGoldColor;
  if (ErrorCode::SUCCESS != _tilePanel.init(cfg.tilePanelCfg,
          numberCounterPanelUtilityCfg)) {
    LOGERR("Error, tilePanelCfg.init() failed");
    return ErrorCode::FAILURE;
  }

  panelPos.y += 165;
  numberCounterPanelUtilityCfg.targetReachedCb =
      [shutdownControllerCb, startAchievementWonCb, startGameWonAnimCb]() {
        shutdownControllerCb();
        startAchievementWonCb(Achievement::TRIPLE_STAR);
        startGameWonAnimCb();
      };
  numberCounterPanelUtilityCfg.pos = panelPos;
  if (ErrorCode::SUCCESS != _crystalPanel.init(cfg.crystalPanelCfg,
          numberCounterPanelUtilityCfg)) {
    LOGERR("Error, _crystalPanel.init() failed");
    return ErrorCode::FAILURE;
  }

  panelPos.y += 175;
  IndicatorPanelUtilityConfig indicatorUtilityCfg;
  indicatorUtilityCfg.indicatorDepletedCb =
      [shutdownControllerCb, startGameLostAnimCb]() {
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
  _tilePanel.draw();
  _crystalPanel.draw();
  _healthPanel.draw();
}

void PanelHandler::modifyHealthIndicator(int32_t damage) {
  _healthPanel.modifyIndicator(-1 * damage);
}

void PanelHandler::onTileRevealed() {
  _tilePanel.increaseCounter(1);
}

void PanelHandler::onCrystalMined() {
  _crystalPanel.increaseCounter(1);
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

  if (nullptr == interface.startAchievementWonAnimCb) {
    LOGERR("Error, nullptr provided for StartAchievementWonAnimCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == interface.fieldMapRevelealedCb) {
    LOGERR("Error, nullptr provided for FieldMapRevelealedCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

