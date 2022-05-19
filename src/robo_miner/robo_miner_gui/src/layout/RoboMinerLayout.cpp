//Corresponding header
#include "robo_miner_gui/layout/RoboMinerLayout.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInterfaces.h"
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInitHelper.h"
#include "robo_miner_gui/helpers/algorithms/FloodFill.h"

using namespace std::placeholders;

ErrorCode RoboMinerLayout::init(const RoboMinerLayoutConfig &cfg,
                              const RoboMinerLayoutOutInterface &outInterface,
                              RoboMinerLayoutInterface &interface) {
  if (ErrorCode::SUCCESS != RoboMinerLayoutInitHelper::init(cfg, outInterface,
          interface.commonLayoutInterface, *this)) {
    LOGERR("Error, RoboMinerLayoutInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  produceInterface(interface);
  return ErrorCode::SUCCESS;
}

void RoboMinerLayout::deinit() {
  _commonLayout.deinit();
}

void RoboMinerLayout::draw() const {
  _commonLayout.draw();
  _panelHandler.draw();
  _crystalHandler.draw();

  _commonLayout.drawSecondLayer();
}

void RoboMinerLayout::handleEvent(const InputEvent &e) {
  _crystalHandler.handleEvent(e);
}

void RoboMinerLayout::produceInterface(RoboMinerLayoutInterface &interface) {
  interface.crystalMinedCb = std::bind(&PanelHandler::onCrystalMined,
      &_panelHandler);
  interface.tileReleavedCb = std::bind(&PanelHandler::onTileRevealed,
      &_panelHandler);
}

