//Corresponding header
#include "robo_collector_controller/helpers/RoboCollectorControllerInitHelper.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "robo_collector_controller/RoboCollectorController.h"
#include "robo_collector_controller/config/RoboCollectorControllerConfig.h"
#include "robo_collector_controller/layout/helpers/RoboCollectorControllerLayoutInterfaces.h"

ErrorCode RoboCollectorControllerInitHelper::init(
    const std::any &cfg, RoboCollectorController &controller) {
  auto err = ErrorCode::SUCCESS;
  const auto parsedCfg = [&cfg, &err]() {
    RoboCollectorControllerConfig localCfg;
    try {
      localCfg = std::any_cast<const RoboCollectorControllerConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<RoboCollectorControllerConfig&> failed, %s",
          e.what());
      err = ErrorCode::FAILURE;
    }
    return localCfg;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing RoboCollectorGuiConfig failed");
    return ErrorCode::FAILURE;
  }

  //allocate memory for the external bridge in order to attach it's callbacks
  controller._controllerExternalBridge =
      std::make_shared<CollectorGuiExternalBridge>();

  RoboCollectorControllerLayoutInterface layoutInterface;
  if (ErrorCode::SUCCESS !=
      initLayout(parsedCfg.layoutCfg, layoutInterface, controller)) {
    LOGERR("Error, initLayout() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS !=
      initControllerExternalBridge(layoutInterface, controller)) {
    LOGERR("initControllerExternalBridge() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCollectorControllerInitHelper::initLayout(
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

  if (ErrorCode::SUCCESS !=
      controller._layout.init(cfg, outInterface, interface)) {
    LOGERR("Error in _layout.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCollectorControllerInitHelper::initControllerExternalBridge(
    const RoboCollectorControllerLayoutInterface &interface,
    RoboCollectorController &controller) {
  CollectorGuiExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = controller._invokeActionEventCb;
  outInterface.enablePlayerInputCb = interface.enablePlayerInputCb;
  outInterface.systemShutdownCb = controller._systemShutdownCb;

  if (ErrorCode::SUCCESS !=
      controller._controllerExternalBridge->init(outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

