//Corresponding header
#include "robo_miner_gui/helpers/RoboMinerGuiInitHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInterfaces.h"
#include "robo_miner_gui/RoboMinerGui.h"
#include "robo_miner_gui/config/RoboMinerGuiConfig.h"

int32_t RoboMinerGuiInitHelper::init(const std::any &cfg, RoboMinerGui &gui) {
  int32_t err = SUCCESS;
  const auto parsedCfg = [&cfg, &err]() {
    RoboMinerGuiConfig localCfg;
    try {
      localCfg = std::any_cast<const RoboMinerGuiConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<RoboMinerGuiConfig&> failed, %s", e.what());
      err = FAILURE;
    }
    return localCfg;
  }();
  if (SUCCESS != err) {
    LOGERR("Error, parsing RoboCollectorGuiConfig failed");
    return FAILURE;
  }

  //allocate memory for the external bridge in order to attach it's callbacks
  gui._controllerExternalBridge =
      std::make_shared<MinerControllerExternalBridge>();

  RoboMinerLayoutInterface layoutInterface;
  if (SUCCESS != initLayout(parsedCfg.layoutCfg, layoutInterface, gui)) {
    LOGERR("Error, initLayout() failed");
    return FAILURE;
  }

  if (SUCCESS != gui._movementWatcher.init(
      layoutInterface.commonLayoutInterface.getPlayerSurroundingTilesCb)) {
    LOGERR("_movementWatcher.init() failed");
    return FAILURE;
  }

  if (SUCCESS != initControllerExternalBridge(layoutInterface, gui)) {
    LOGERR("initControllerExternalBridge() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboMinerGuiInitHelper::initLayout(const RoboMinerLayoutConfig &cfg,
                                           RoboMinerLayoutInterface &interface,
                                           RoboMinerGui &gui) {
  using namespace std::placeholders;

  RoboMinerLayoutOutInterface outInterface;
  outInterface.collisionWatcher = &gui._collisionWatcher;
  outInterface.finishRobotActCb =
      std::bind(&RoboMinerGui::onRobotTurnFinish, &gui, _1, _2);
  outInterface.shutdownGameCb = std::bind(
      &MinerControllerExternalBridge::publishShutdownController,
      gui._controllerExternalBridge.get());

  if (SUCCESS != gui._layout.init(cfg, outInterface, interface)) {
    LOGERR("Error in _layout.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboMinerGuiInitHelper::initControllerExternalBridge(
    const RoboMinerLayoutInterface &interface, RoboMinerGui &gui) {
  MinerControllerExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = gui._invokeActionEventCb;
  outInterface.robotActCb =
      interface.commonLayoutInterface.playerRobotActInterface.actCb;
  outInterface.systemShutdownCb = gui._systemShutdownCb;
  outInterface.movementWatcher = &gui._movementWatcher;

  if (SUCCESS != gui._controllerExternalBridge->init(outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return FAILURE;
  }

  return SUCCESS;
}

