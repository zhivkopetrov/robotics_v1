//Corresponding header
#include "robo_collector_controller/helpers/RoboCollectorControllerInitHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_controller/RoboCollectorController.h"
#include "robo_collector_controller/config/RoboCollectorControllerConfig.h"
#include "robo_collector_controller/layout/helpers/RoboCollectorControllerLayoutInterfaces.h"

int32_t RoboCollectorControllerInitHelper::init(
    const std::any &cfg, RoboCollectorController &controller) {
  int32_t err = SUCCESS;
  const auto parsedCfg = [&cfg, &err]() {
    RoboCollectorControllerConfig localCfg;
    try {
      localCfg = std::any_cast<const RoboCollectorControllerConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<RoboCollectorControllerConfig&> failed, %s",
          e.what());
      err = FAILURE;
    }
    return localCfg;
  }();
  if (SUCCESS != err) {
    LOGERR("Error, parsing RoboCollectorGuiConfig failed");
    return FAILURE;
  }

  //allocate memory for the external bridge in order to attach it's callbacks
  controller._controllerExternalBridge =
      std::make_shared<CollectorGuiExternalBridge>();

  RoboCollectorControllerLayoutInterface layoutInterface;
  if (SUCCESS != initLayout(parsedCfg.layoutCfg, layoutInterface, controller)) {
    LOGERR("Error, initLayout() failed");
    return FAILURE;
  }

  if (SUCCESS != initControllerExternalBridge(layoutInterface, controller)) {
    LOGERR("initControllerExternalBridge() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorControllerInitHelper::initLayout(
    const RoboCollectorControllerLayoutConfig &cfg,
    RoboCollectorControllerLayoutInterface &interface,
    RoboCollectorController &controller) {
  using namespace std::placeholders;

  RoboCollectorControllerLayoutOutInterface outInterface;
  const auto guiExternalBridgeRawPointer =
      controller._controllerExternalBridge.get();
  outInterface.robotActCb = std::bind(
      &CollectorGuiExternalBridge::publishRobotAct,
      guiExternalBridgeRawPointer, _1);
  outInterface.helpActivatedCb = std::bind(
      &CollectorGuiExternalBridge::publishToggleHelp,
      guiExternalBridgeRawPointer);
  outInterface.settingActivatedCb = std::bind(
      &CollectorGuiExternalBridge::publishToggleSettings,
      guiExternalBridgeRawPointer);

  if (SUCCESS != controller._layout.init(cfg, outInterface, interface)) {
    LOGERR("Error in _layout.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorControllerInitHelper::initControllerExternalBridge(
    const RoboCollectorControllerLayoutInterface &interface,
    RoboCollectorController &controller) {
  CollectorGuiExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = controller._invokeActionEventCb;
  outInterface.enablePlayerInputCb = interface.enablePlayerInputCb;
  outInterface.systemShutdownCb = controller._systemShutdownCb;

  if (SUCCESS != controller._controllerExternalBridge->init(outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return FAILURE;
  }

  return SUCCESS;
}

