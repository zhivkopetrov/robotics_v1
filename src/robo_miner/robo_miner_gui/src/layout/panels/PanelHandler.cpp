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
  auto panelPos = Point(1250, 390);
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
  _healthPanel.draw();
}

void PanelHandler::decreaseHealthIndicator(int32_t damage) {
  _healthPanel.decreaseIndicator(damage);
}
