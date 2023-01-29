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
    const std::any &cfg, UrControlBloom &bloom) {
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
  bloom._dashboardProvider = std::make_shared<DashboardProvider>();
  bloom._externalBridge = 
    std::make_shared<UrControlCommonExternalBridge>(NODE_NAME);

  UrControlCommonLayoutInterface layoutInterface;
  if (ErrorCode::SUCCESS != initLayout(parsedCfg.commonLayoutCfg, 
      layoutInterface, bloom)) {
    LOGERR("Error, initLayout() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initDashboardHelper(layoutInterface, bloom)) {
    LOGERR("initDashboardHelper() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initUrControlBloomExternalBridge(
          parsedCfg.externalBridgeCfg, layoutInterface, bloom)) {
    LOGERR("initControllerExternalBridge() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initStateMachine(bloom)) {
    LOGERR("initStateMachine() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initLayout(
    const UrControlCommonLayoutConfig &cfg,
    UrControlCommonLayoutInterface &layoutInterface, UrControlBloom &bloom) {
  UrControlCommonLayoutOutInterface layoutOutInterface;
  const auto externalBridgeRawPointer = bloom._externalBridge.get();
  layoutOutInterface.publishURScriptCb = std::bind(
      &UrControlCommonExternalBridge::publishURScript, 
      externalBridgeRawPointer, _1);

  const auto dashboardProviderRawPointer = bloom._dashboardProvider.get();
  layoutOutInterface.invokeDashboardCb = std::bind(
      &DashboardProvider::invokeDashboard, dashboardProviderRawPointer, _1);

  if (ErrorCode::SUCCESS != bloom._layout.init(cfg, layoutOutInterface,
          layoutInterface)) {
    LOGERR("Error in _layout.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initDashboardHelper(
    const UrControlCommonLayoutInterface &layoutInterface, 
    UrControlBloom &bloom) {
  DashboardProviderOutInterface outInterface;
  outInterface.invokeActionEventCb = bloom._invokeActionEventCb;
  outInterface.robotModeChangeCb = layoutInterface.robotModeChangeCb;
  outInterface.safetyModeChangeCb = layoutInterface.safetyModeChangeCb;

  if (ErrorCode::SUCCESS != bloom._dashboardProvider->init(outInterface)) {
    LOGERR("Error in _dashboardProvider.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initUrControlBloomExternalBridge(
    const UrContolBloomExternalBridgeConfig &cfg,
    const UrControlCommonLayoutInterface &layoutInterface, 
    UrControlBloom &bloom) {
  UrControlCommonExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = bloom._invokeActionEventCb;
  outInterface.robotModeChangeCb = layoutInterface.robotModeChangeCb;
  outInterface.safetyModeChangeCb = layoutInterface.safetyModeChangeCb;

  if (ErrorCode::SUCCESS != 
        bloom._externalBridge->init(cfg.commonConfig, outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initStateMachine(UrControlBloom &bloom) {
  StateMachine &stateMachine = bloom._stateMachine;

  std::vector<StateDescription> stateDescriptions;
  StateDescription state;
  state.name = "Recovery";
  state.onEnter = [](){
    LOG("Hello from Recovery state");
  };
  state.onExit = [](){
    LOG("Bye from Recovery state");
  };
  stateDescriptions.push_back(state);

  state.name = "Bloom1";
  state.onEnter = [](){
    LOG("Hello from Bloom1 state");
  };
  state.onExit = [](){
    LOG("Bye from Bloom1 state");
  };
  stateDescriptions.push_back(state);

  state.name = "SideQuest";
  state.onEnter = [](){
    LOG("Hello from SideQuest state");
  };
  state.onExit = [](){
    LOG("Bye from SideQuest state");
  };
  stateDescriptions.push_back(state);

  std::vector<StateTransitions> stateTransitions;
  StateTransitions transition;
  transition.stateName = "Recovery";
  transition.transitions.insert("Bloom1");
  stateTransitions.push_back(transition);

  transition.stateName = "Bloom1";
  transition.transitions.insert("SideQuest");
  stateTransitions.push_back(transition);

  if (ErrorCode::SUCCESS != stateMachine.init(
    StateLogging::ENABLED, stateDescriptions, stateTransitions)) {
    LOGERR("Error stateMachine.init()");
    return ErrorCode::FAILURE;
  }

  //test correct transition
  stateMachine.changeState("Recovery");

  //test bad transition
  stateMachine.changeState("SideQuest");

  //test correct transition
  stateMachine.changeState("Bloom1");

  //test bad transition
  stateMachine.changeState("Recovery");

  //test correct transition
  stateMachine.changeState("SideQuest");

  return ErrorCode::SUCCESS;
}
