//Corresponding header
#include "robo_cleaner_gui/helpers/RoboCleanerGuiInitHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/layout/helpers/RoboCleanerLayoutInterfaces.h"
#include "robo_cleaner_gui/RoboCleanerGui.h"
#include "robo_cleaner_gui/config/RoboCleanerGuiConfig.h"

int32_t RoboCleanerGuiInitHelper::init(const std::any &cfg, RoboCleanerGui &gui) {
  int32_t err = SUCCESS;
  const auto parsedCfg = [&cfg, &err]() {
    RoboCleanerGuiConfig localCfg;
    try {
      localCfg = std::any_cast<const RoboCleanerGuiConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<RoboCleanerGuiConfig&> failed, %s", e.what());
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
      std::make_shared<CleanerControllerExternalBridge>();

  RoboCleanerLayoutInterface layoutInterface;
  if (SUCCESS != initLayout(parsedCfg.layoutCfg, layoutInterface, gui)) {
    LOGERR("Error, initLayout() failed");
    return FAILURE;
  }

  if (SUCCESS != initControllerExternalBridge(layoutInterface, gui)) {
    LOGERR("initControllerExternalBridge() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCleanerGuiInitHelper::initLayout(const RoboCleanerLayoutConfig &cfg,
                                           RoboCleanerLayoutInterface &interface,
                                           RoboCleanerGui &gui) {
  using namespace std::placeholders;

  RoboCleanerLayoutOutInterface outInterface;
  outInterface.collisionWatcher = &gui._collisionWatcher;
  outInterface.finishRobotActCb =
      std::bind(&RoboCleanerGui::onRobotTurnFinish, &gui, _1, _2);
  outInterface.shutdownGameCb = std::bind(
      &CleanerControllerExternalBridge::publishShutdownController,
      gui._controllerExternalBridge.get());

  if (SUCCESS != gui._layout.init(cfg, outInterface, interface)) {
    LOGERR("Error in _layout.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCleanerGuiInitHelper::initControllerExternalBridge(
    const RoboCleanerLayoutInterface &interface, RoboCleanerGui &gui) {
  CleanerControllerExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = gui._invokeActionEventCb;
  outInterface.robotActCb =
      interface.commonLayoutInterface.playerRobotActInterface.actCb;

  if (SUCCESS != gui._controllerExternalBridge->init(outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return FAILURE;
  }

  return SUCCESS;
}

