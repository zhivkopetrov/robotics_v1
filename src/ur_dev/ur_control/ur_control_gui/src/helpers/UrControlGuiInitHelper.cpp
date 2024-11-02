//Corresponding header
#include "ur_control_gui/helpers/UrControlGuiInitHelper.h"

//System headers

//Other libraries headers
#include "ur_control_common/layout/helpers/UrControlCommonLayoutInterfaces.h"
#include "utils/log/Log.h"

//Own components headers
#include "ur_control_gui/UrControlGui.h"
#include "ur_control_gui/config/UrControlGuiConfig.h"

using namespace std::placeholders;

namespace {
  constexpr auto NODE_NAME = "UrControlGuiExternalBridge";
}

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
  gui._guiExternalBridge = 
    std::make_shared<UrControlCommonExternalBridge>(NODE_NAME);

  UrControlCommonLayoutInterface layoutInterface;
  if (ErrorCode::SUCCESS != initLayout(parsedCfg.commonLayoutCfg, 
      layoutInterface, gui)) {
    LOGERR("Error, initLayout() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initDashboardHelper(layoutInterface, gui)) {
    LOGERR("initDashboardHelper() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initUrControlGuiExternalBridge(
          parsedCfg.externalBridgeCfg, layoutInterface, gui)) {
    LOGERR("initControllerExternalBridge() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlGuiInitHelper::initLayout(
    const UrControlCommonLayoutConfig &cfg,
    UrControlCommonLayoutInterface &layoutInterface, UrControlGui &gui) {
  UrControlCommonLayoutOutInterface layoutOutInterface;
  const auto guiExternalBridgeRawPointer = gui._guiExternalBridge.get();
  layoutOutInterface.publishURScriptCb = std::bind(
      &UrControlCommonExternalBridge::publishURScript, 
      guiExternalBridgeRawPointer, _1);

  const auto dashboardProviderRawPointer = gui._dashboardProvider.get();
  layoutOutInterface.invokeDashboardServiceCb = std::bind(
      &DashboardProvider::invokeDashboard, dashboardProviderRawPointer, _1);

  if (ErrorCode::SUCCESS != gui._layout.init(cfg, layoutOutInterface,
          layoutInterface)) {
    LOGERR("Error in _layout.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlGuiInitHelper::initDashboardHelper(
    const UrControlCommonLayoutInterface &layoutInterface, UrControlGui &gui) {
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
    const UrControlCommonLayoutInterface &layoutInterface, UrControlGui &gui) {
  UrControlCommonExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = gui._invokeActionEventCb;
  outInterface.robotModeChangeCb = layoutInterface.robotModeChangeCb;
  outInterface.safetyModeChangeCb = layoutInterface.safetyModeChangeCb;

  if (ErrorCode::SUCCESS != 
        gui._guiExternalBridge->init(cfg.commonConfig, outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

