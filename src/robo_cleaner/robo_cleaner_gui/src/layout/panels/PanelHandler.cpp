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
  auto panelPos = Point(1250, 390);
  if (SUCCESS != _healthPanel.init(
      cfg.healthPanelCfg, interface.startGameLostAnimCb, panelPos)) {
    LOGERR("Error, _healthPanel.init() failed");
    return FAILURE;
  }

  panelPos.y += 95;
  if (SUCCESS != _energyPanel.init(
      cfg.energyPanelCfg, interface.energyDepletedCb, panelPos)) {
    LOGERR("Error, _energyPanel.init() failed");
    return FAILURE;
  }

  return SUCCESS;
}

void PanelHandler::draw() const {
  _healthPanel.draw();
  _energyPanel.draw();
}

void PanelHandler::decreaseHealthIndicator(int32_t delta) {
  _healthPanel.decreaseIndicator(delta);
}

void PanelHandler::decreaseEnergyIndicator(int32_t delta) {
  _energyPanel.decreaseIndicator(delta);
}

