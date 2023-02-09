//Corresponding header
#include "ur_control_bloom/helpers/UrControlBloomInitHelper.h"

//System headers

//Other libraries headers

#include "ur_control_common/layout/helpers/UrControlCommonLayoutInterfaces.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/UrControlBloom.h"
#include "ur_control_bloom/config/UrControlBloomConfig.h"
#include "ur_control_bloom/defines/UrControlBloomDefines.h"
#include "ur_control_bloom/motion/BloomMotionSequence.h"
#include "ur_control_bloom/motion/JengaMotionSequence.h"

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
  if (ErrorCode::SUCCESS != 
      initLayout(parsedCfg.layoutCfg, layoutInterface, bloom)) {
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

  if (ErrorCode::SUCCESS != initMotionExecutor(parsedCfg, bloom)) {
    LOGERR("initMotionExecutor() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initStateMachine(bloom)) {
    LOGERR("initStateMachine() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initLayout(
    const UrControlBloomLayoutConfig &cfg,
    UrControlCommonLayoutInterface &layoutInterface, UrControlBloom &bloom) {
  UrControlCommonLayoutOutInterface layoutOutInterface;
  const auto externalBridgeRawPointer = bloom._externalBridge.get();
  layoutOutInterface.publishURScriptCb = std::bind(
      &UrControlCommonExternalBridge::publishURScript, 
      externalBridgeRawPointer, _1);

  const auto dashboardProviderRawPointer = bloom._dashboardProvider.get();
  layoutOutInterface.invokeDashboardServiceCb = std::bind(
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

ErrorCode UrControlBloomInitHelper::initMotionExecutor(
  const UrControlBloomConfig &cfg, UrControlBloom &bloom) {
  MotionSequenceExecutorOutInterface outInterface;
  const auto externalBridgeRawPointer = bloom._externalBridge.get();
  outInterface.publishURScriptCb = std::bind(
    &UrControlCommonExternalBridge::publishURScript, 
    externalBridgeRawPointer, _1);

  outInterface.invokeURScriptServiceCb = std::bind(
    &UrControlCommonExternalBridge::invokeURScriptService, 
    externalBridgeRawPointer, _1);

  outInterface.invokeURScriptPreemptServiceCb = std::bind(
    &UrControlCommonExternalBridge::invokeURScriptPreemptService, 
    externalBridgeRawPointer);

  outInterface.invokeActionEventCb = bloom._invokeActionEventCb;

  if (ErrorCode::SUCCESS != bloom._motionExecutor.init(outInterface)) {
    LOGERR("Error in _motionExecutor.init()");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != 
        initBloomMotionSequence(cfg.bloomMotionSequenceCfg, bloom)) {
    LOGERR("Error in initBloomMotionSequence()");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != 
        initJengaMotionSequence(cfg.jengaMotionSequenceCfg, bloom)) {
    LOGERR("Error in initJengaMotionSequence()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initBloomMotionSequence(
    const BloomMotionSequenceConfig &cfg, UrControlBloom &bloom) {
  auto bloomMotionSequence = std::make_unique<BloomMotionSequence>(
    Motion::BLOOM_MOTION_SEQUENCE_NAME, Motion::BLOOM_MOTION_ID);

  if (ErrorCode::SUCCESS != bloomMotionSequence->init(cfg)) {
    LOGERR("Error in bloomMotionSequence->init()");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != 
    bloom._motionExecutor.addSequence(std::move(bloomMotionSequence))) {
    LOGERR("Error in motionExecutor.addSequence()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initJengaMotionSequence(
  const JengaMotionSequenceConfig &cfg, UrControlBloom &bloom) {
  auto jengaMotionSequence = std::make_unique<JengaMotionSequence>(
    Motion::JENGA_MOTION_SEQUENCE_NAME, Motion::JENGA_MOTION_ID);

  if (ErrorCode::SUCCESS != jengaMotionSequence->init(cfg)) {
    LOGERR("Error in jengaMotionSequence->init()");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != 
    bloom._motionExecutor.addSequence(std::move(jengaMotionSequence))) {
    LOGERR("Error in motionExecutor.addSequence()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initStateMachine(UrControlBloom &bloom) {
  StateMachine &sm = bloom._stateMachine;

  std::vector<StateDescription> stateDescriptions;
  stateDescriptions.reserve(BloomState::STATES_COUNT);

  StateDescription state;
  state.name = BloomState::INIT;
  state.onEnter = std::bind(&UrControlBloom::enterInitState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitInitState, &bloom);
  stateDescriptions.push_back(state);

  state.name = BloomState::IDLE;
  state.onEnter = std::bind(&UrControlBloom::enterIdleState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitIdleState, &bloom);
  stateDescriptions.push_back(state);

  state.name = BloomState::BLOOM;
  state.onEnter = std::bind(&UrControlBloom::enterBloomState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitBloomState, &bloom);
  stateDescriptions.push_back(state);

  state.name = BloomState::BLOOM_RECOVERY;
  state.onEnter = std::bind(&UrControlBloom::enterBloomRecoveryState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitBloomRecoveryState, &bloom);
  stateDescriptions.push_back(state);

  state.name = BloomState::JENGA;
  state.onEnter = std::bind(&UrControlBloom::enterJengaState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitJengaState, &bloom);
  stateDescriptions.push_back(state);

  state.name = BloomState::JENGA_RECOVERY;
  state.onEnter = std::bind(&UrControlBloom::enterJengaRecoveryState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitJengaRecoveryState, &bloom);
  stateDescriptions.push_back(state);

  std::vector<StateTransitions> stateTransitions;
  StateTransitions transition;
  transition.stateName = BloomState::INIT;
  transition.transitions.insert(BloomState::BLOOM_RECOVERY);
  transition.transitions.insert(BloomState::JENGA_RECOVERY);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::BLOOM_RECOVERY;
  transition.transitions.insert(BloomState::IDLE);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::JENGA_RECOVERY;
  transition.transitions.insert(BloomState::IDLE);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::IDLE;
  transition.transitions.insert(BloomState::BLOOM);
  transition.transitions.insert(BloomState::JENGA);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::BLOOM;
  transition.transitions.insert(BloomState::JENGA);
  transition.transitions.insert(BloomState::IDLE);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::JENGA;
  transition.transitions.insert(BloomState::BLOOM);
  transition.transitions.insert(BloomState::IDLE);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  if (ErrorCode::SUCCESS != sm.init(
    StateLogging::ENABLED, stateDescriptions, stateTransitions)) {
    LOGERR("Error in stateMachine.init()");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != sm.start(BloomState::INIT)) {
    LOGERR("Error in stateMachine.start()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}