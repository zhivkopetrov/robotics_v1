//Corresponding header
#include "ur_control_gui/helpers/UrControlGuiInitHelper.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/UrControlGui.h"
#include "ur_control_gui/config/UrControlGuiConfig.h"
#include "ur_control_gui/layout/helpers/UrControlGuiLayoutInterfaces.h"

using namespace std::placeholders;

ErrorCode UrControlGuiInitHelper::init(const std::any &cfg, UrControlGui &gui) {
  auto err = ErrorCode::SUCCESS;
  const auto parsedCfg = [&cfg, &err]() {
    UrControlGuiConfig localCfg;
    try {
      localCfg = std::any_cast<const UrControlGuiConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<UrControlGuiConfig&> failed, %s", e.what());
      err = ErrorCode::FAILURE;
    }
    return localCfg;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing UrControlGuiConfig failed");
    return ErrorCode::FAILURE;
  }

  //allocate memory for the ROS nodes in order to attach it's callbacks
  gui._dashboardProvider = std::make_shared<DashboardProvider>();
  gui._guiExternalBridge = std::make_shared<UrControlGuiExternalBridge>();

  UrControlGuiLayoutInterface layoutInterface;
  if (ErrorCode::SUCCESS != initLayout(parsedCfg.layoutCfg, layoutInterface,
          gui)) {
    LOGERR("Error, initLayout() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initDashboardHelper(layoutInterface, gui)) {
    LOGERR("initDashboardHelper() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initUrControlGuiExternalBridge(
          parsedCfg.urContolGuiExternalBridgeCfg, layoutInterface, gui)) {
    LOGERR("initControllerExternalBridge() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlGuiInitHelper::initLayout(
    const UrControlGuiLayoutConfig &cfg,
    UrControlGuiLayoutInterface &layoutInterface, UrControlGui &gui) {
  UrControlGuiLayoutOutInterface layoutOutInterface;
  const auto guiExternalBridgeRawPointer = gui._guiExternalBridge.get();
  layoutOutInterface.publishURScriptCb = std::bind(
      &UrControlGuiExternalBridge::publishURScript, guiExternalBridgeRawPointer,
      _1);

  const auto dashboardProviderRawPointer = gui._dashboardProvider.get();
  layoutOutInterface.invokeDashboardCb = std::bind(
      &DashboardProvider::invokeDashboard, dashboardProviderRawPointer, _1);

  if (ErrorCode::SUCCESS != gui._layout.init(cfg, layoutOutInterface,
          layoutInterface)) {
    LOGERR("Error in _layout.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlGuiInitHelper::initDashboardHelper(
    const UrControlGuiLayoutInterface &layoutInterface, UrControlGui &gui) {
  DashboardProviderOutInterface outInterface;
  outInterface.invokeActionEventCb = gui._invokeActionEventCb;
  outInterface.robotModeChangeCb = layoutInterface.robotModeChangeCb;
  outInterface.safetyModeChangeCb = layoutInterface.safetyModeChangeCb;

  if (ErrorCode::SUCCESS != gui._dashboardProvider->init(outInterface)) {
    LOGERR("Error in _dashboardProvider.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlGuiInitHelper::initUrControlGuiExternalBridge(
    const UrContolGuiExternalBridgeConfig &cfg,
    const UrControlGuiLayoutInterface &layoutInterface, UrControlGui &gui) {
  UrControlGuiExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = gui._invokeActionEventCb;
  outInterface.robotModeChangeCb = layoutInterface.robotModeChangeCb;
  outInterface.safetyModeChangeCb = layoutInterface.safetyModeChangeCb;

  if (ErrorCode::SUCCESS != gui._guiExternalBridge->init(cfg, outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

