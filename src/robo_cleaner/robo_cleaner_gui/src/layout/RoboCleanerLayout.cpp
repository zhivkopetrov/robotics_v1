//Corresponding header
#include "robo_cleaner_gui/layout/RoboCleanerLayout.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/layout/helpers/RoboCleanerLayoutInterfaces.h"
#include "robo_cleaner_gui/layout/helpers/RoboCleanerLayoutInitHelper.h"

ErrorCode RoboCleanerLayout::init(
    const RoboCleanerLayoutConfig &cfg,
    const RoboCleanerLayoutOutInterface &outInterface,
    RoboCleanerLayoutInterface &interface) {
  if (ErrorCode::SUCCESS != RoboCleanerLayoutInitHelper::init(cfg, outInterface,
          interface.commonLayoutInterface, *this)) {
    LOGERR("Error, RoboCleanerLayoutInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  produceInterface(interface);
  return ErrorCode::SUCCESS;
}

void RoboCleanerLayout::deinit() {
  _commonLayout.deinit();
}

void RoboCleanerLayout::draw() const {
  _commonLayout.draw();
  _entityHandler.draw();
  _panelHandler.draw();

  _commonLayout.drawSecondLayer();
  _commonLayout.drawThirdLayer();
}

void RoboCleanerLayout::onEnergyDepleted() {
  LOGR("Energy depleted");
}

void RoboCleanerLayout::produceInterface(
    RoboCleanerLayoutInterface &interface) {
  using namespace std::placeholders;

  interface.modifyRubbishWidgetCb = std::bind(
      &EntityHandler::modifyRubbishWidget, &_entityHandler, _1, _2);
  interface.tileReleavedCb = std::bind(&PanelHandler::onTileRevealed,
      &_panelHandler);
  interface.tileCleanedCb = std::bind(&PanelHandler::onTileCleaned,
      &_panelHandler);
}

