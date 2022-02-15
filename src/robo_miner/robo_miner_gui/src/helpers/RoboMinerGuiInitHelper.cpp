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

  RoboMinerLayoutInterface layoutInterface;
  if (SUCCESS != initLayout(parsedCfg.layoutCfg, layoutInterface, gui)) {
    LOGERR("Error, initLayout() failed");
    return FAILURE;
  }

  if (SUCCESS != initControllerExternalBridge(gui)) {
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
      std::bind(&RoboMinerGui::onRobotTurnFinish, &gui, _1);

  if (SUCCESS != gui._layout.init(cfg, outInterface, interface)) {
    LOGERR("Error in _layout.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboMinerGuiInitHelper::initControllerExternalBridge(
    RoboMinerGui &gui) {
  MinerControllerExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = gui._invokeActionEventCb;

  gui._controllerExternalBridge =
      std::make_shared<MinerControllerExternalBridge>();
  if (SUCCESS != gui._controllerExternalBridge->init(outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return FAILURE;
  }

  return SUCCESS;
}

