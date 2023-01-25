//Corresponding header
#include "ur_control_bloom/helpers/UrControlBloomInitHelper.h"

//System headers

//Other libraries headers
#include "ur_control_common/layout/helpers/UrControlCommonLayoutInterfaces.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/UrControlBloom.h"
#include "ur_control_bloom/config/UrControlBloomConfig.h"

using namespace std::placeholders;

namespace {
  constexpr auto NODE_NAME = "UrControlBloomExternalBridge";
}

ErrorCode UrControlBloomInitHelper::init(
    const std::any &cfg, UrControlBloom &gui) {
  auto err = ErrorCode::SUCCESS;
  const auto parsedCfg = [&cfg, &err]() {
    UrControlBloomConfig localCfg;
    try {
      localCfg = std::any_cast<const UrControlBloomConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<UrControlBloomConfig&> failed, %s", e.what());
      err = ErrorCode::FAILURE;
    }
    return localCfg;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing UrControlBloomConfig failed");
    return ErrorCode::FAILURE;
  }

  //allocate memory for the ROS nodes in order to attach it's callbacks
  gui._dashboardProvider = std::make_shared<DashboardProvider>();
  gui._externalBridge = 
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

  if (ErrorCode::SUCCESS != initUrControlBloomExternalBridge(
          parsedCfg.externalBridgeCfg, layoutInterface, gui)) {
    LOGERR("initControllerExternalBridge() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initLayout(
    const UrControlCommonLayoutConfig &cfg,
    UrControlCommonLayoutInterface &layoutInterface, UrControlBloom &gui) {
  UrControlCommonLayoutOutInterface layoutOutInterface;
  const auto guiExternalBridgeRawPointer = gui._externalBridge.get();
  layoutOutInterface.publishURScriptCb = std::bind(
      &UrControlCommonExternalBridge::publishURScript, 
      guiExternalBridgeRawPointer, _1);

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

ErrorCode UrControlBloomInitHelper::initDashboardHelper(
    const UrControlCommonLayoutInterface &layoutInterface, UrControlBloom &gui) {
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

ErrorCode UrControlBloomInitHelper::initUrControlBloomExternalBridge(
    const UrContolBloomExternalBridgeConfig &cfg,
    const UrControlCommonLayoutInterface &layoutInterface, UrControlBloom &gui) {
  UrControlCommonExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = gui._invokeActionEventCb;
  outInterface.robotModeChangeCb = layoutInterface.robotModeChangeCb;
  outInterface.safetyModeChangeCb = layoutInterface.safetyModeChangeCb;

  if (ErrorCode::SUCCESS != 
        gui._externalBridge->init(cfg.commonConfig, outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

