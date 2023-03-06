//Corresponding header
#include "ur_control_bloom/helpers/UrControlBloomInitHelper.h"

//System headers

//Other libraries headers
#include "urscript_common/gripper/GripperStructs.h"
#include "ur_control_common/layout/helpers/UrControlCommonLayoutInterfaces.h"
#include "ur_control_common/layout/entities/button_handler/ButtonHandlerInterfaces.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/UrControlBloom.h"
#include "ur_control_bloom/config/UrControlBloomConfig.h"
#include "ur_control_bloom/defines/UrControlBloomDefines.h"
#include "ur_control_bloom/motion/BloomMotionSequence.h"
#include "ur_control_bloom/motion/JengaMotionSequence.h"
#include "ur_control_bloom/motion/ParkMotionSequence.h"

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

  //allocate memory for objects in order to attach it's callbacks
  bloom._urScriptBuilder = std::make_shared<UrScriptBuilder>();
  bloom._dashboardProvider = std::make_shared<DashboardProvider>();
  bloom._externalBridge = 
    std::make_shared<UrControlCommonExternalBridge>(NODE_NAME);
  bloom._stateFileHandler = std::make_shared<StateFileHandler>();

  if (ErrorCode::SUCCESS != 
      initUrScriptBuilder(parsedCfg.urScriptBuilderCfg, bloom)) {
    LOGERR("initUrScriptBuilder() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != 
      initStateFileHandler(parsedCfg.stateFilePath, bloom)) {
    LOGERR("initStateFileHandler() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != 
      initMotionExecutor(parsedCfg.motionSequenceCfg, bloom)) {
    LOGERR("initMotionExecutor() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initStateMachine(bloom)) {
    LOGERR("initStateMachine() failed");
    return ErrorCode::FAILURE;
  }

  UrControlBloomLayoutInterface layoutInterface;
  if (ErrorCode::SUCCESS != 
      initLayout(parsedCfg.layoutCfg, layoutInterface, bloom)) {
    LOGERR("Error, initLayout() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != 
        initDashboardHelper(layoutInterface.commonInterface, bloom)) {
    LOGERR("initDashboardHelper() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initUrControlBloomExternalBridge(
        parsedCfg.externalBridgeCfg, layoutInterface.commonInterface, bloom)) {
    LOGERR("initUrControlBloomExternalBridge() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initLayout(
    const UrControlBloomLayoutConfig &cfg,
    UrControlBloomLayoutInterface &layoutInterface, UrControlBloom &bloom) {
  UrControlBloomLayoutOutInterface layoutOutInterface;
  UrControlCommonLayoutOutInterface& commonOutInterface = 
    layoutOutInterface.commonOutInterface;

  const auto externalBridgeRawPointer = bloom._externalBridge.get();
  commonOutInterface.publishURScriptCb = std::bind(
      &UrControlCommonExternalBridge::publishURScript, 
      externalBridgeRawPointer, _1);

  const auto dashboardProviderRawPointer = bloom._dashboardProvider.get();
  commonOutInterface.invokeDashboardServiceCb = std::bind(
      &DashboardProvider::invokeDashboard, dashboardProviderRawPointer, _1);

  CustomActionButtonHandlerCbs customActionButtonHandlerCbs;
  populateCustomActionButtonHandlerCbs(customActionButtonHandlerCbs, bloom);
  commonOutInterface.buttonHandlerAdditionalOutInterface = 
    customActionButtonHandlerCbs;

  if (ErrorCode::SUCCESS != bloom._layout.init(cfg, layoutOutInterface,
          layoutInterface)) {
    LOGERR("Error in _layout.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

//TODO populate this method accordingly
void UrControlBloomInitHelper::populateCustomActionButtonHandlerCbs(
    CustomActionButtonHandlerCbs& outCbs, UrControlBloom &bloom) {
  UrScriptCommandContainer cmdContainer;
  UrScriptPayload cmdPayload;
  UrScriptBuilder& builder = *bloom._urScriptBuilder;
  const InvokeURScriptServiceCb invokeURScriptServiceCb = std::bind(
      &UrControlCommonExternalBridge::invokeURScriptService, 
      bloom._externalBridge.get(), _1);

  CustomActionButtonCbs& gripperCbs = outCbs.gripperButtonCbs;
  gripperCbs.resize(GRIPPER_BUTTONS_COUNT);

  { //gripperCbs[ACTIVATE_GRIPPER_IDX]
    auto gripperActiveCommand = std::make_unique<GripperActivateCommand>();
    constexpr int32_t gripperSpeedPercent = 100;
    auto gripperSpeedCommand = std::make_unique<GripperParamCommand>(
      GripperParamType::SPEED, gripperSpeedPercent);
    constexpr int32_t gripperForcePercent = 100;
    auto gripperForceCommand = std::make_unique<GripperParamCommand>(
      GripperParamType::FORCE, gripperForcePercent);
    cmdContainer.addCommand(std::move(gripperActiveCommand))
                .addCommand(std::move(gripperSpeedCommand))
                .addCommand(std::move(gripperForceCommand));
    cmdPayload = builder.construct(Gripper::ACTIVATE_NAME, cmdContainer);
    gripperCbs[ACTIVATE_GRIPPER_IDX] = [cmdPayload, invokeURScriptServiceCb](){
      invokeURScriptServiceCb(cmdPayload);
    };
  }

  { //gripperCbs[OPEN_GRIPPER_IDX]
    auto openGripperCommand = 
      std::make_unique<GripperActuateCommand>(GripperActuateType::OPEN);
    cmdContainer.addCommand(std::move(openGripperCommand));
    cmdPayload = builder.construct(Gripper::OPEN_NAME, cmdContainer);
    gripperCbs[OPEN_GRIPPER_IDX] = [cmdPayload, invokeURScriptServiceCb](){
      invokeURScriptServiceCb(cmdPayload);
    };
  }

  { //gripperCbs[CLOSE_GRIPPER_IDX]
    auto openGripperCommand = 
      std::make_unique<GripperActuateCommand>(GripperActuateType::CLOSE);
    cmdContainer.addCommand(std::move(openGripperCommand));
    cmdPayload = builder.construct(Gripper::CLOSE_NAME, cmdContainer);
    gripperCbs[CLOSE_GRIPPER_IDX] = [cmdPayload, invokeURScriptServiceCb](){
      invokeURScriptServiceCb(cmdPayload);
    };
  }

  CustomActionButtonCbs& commandCbs = outCbs.commandButtonCbs;
  commandCbs.resize(CUSTOM_ACTION_BUTTONS_COUNT);

  commandCbs[PARK_IDX] = [&bloom](){
    bloom._stateMachine.changeState(BloomState::PARK);
  };

  commandCbs[JENGA_IDX] = [&bloom](){
    bloom._stateMachine.changeState(BloomState::JENGA_RECOVERY);
  };

  commandCbs[BLOOM_RANDOMIZED_IDX] = [&bloom](){
    bloom.executeRandomizedBloomStrategy();
  };

  commandCbs[BLOOM_1ST_IDX] = [&bloom](){
    constexpr int32_t strategyId = 
      getEnumValue(Motion::Bloom::TransportStrategy::BASIC);
    bloom.executeBloomStrategy(strategyId);
  };

  commandCbs[BLOOM_2ND_IDX] = [&bloom](){
    constexpr int32_t strategyId = 
      getEnumValue(Motion::Bloom::TransportStrategy::FULL_ROTATION);
    bloom.executeBloomStrategy(strategyId);
  };

  commandCbs[BLOOM_3RD_IDX] = [&bloom](){
    constexpr int32_t strategyId = 
      getEnumValue(Motion::Bloom::TransportStrategy::TWIST);
    bloom.executeBloomStrategy(strategyId);
  };

  commandCbs[GRACEFUL_STOP_IDX] = [&bloom](){
    bloom.executeGracefulStopAndTransitionToIdle();
  };

  commandCbs[ABORT_MOTION_IDX] = [&bloom](){
    bloom.executeAbortMotion();
  };
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

  ErrorCode UrControlBloomInitHelper::initUrScriptBuilder(
    const UrScriptBuilderConfig &cfg, UrControlBloom &bloom) {
  if (ErrorCode::SUCCESS != bloom._urScriptBuilder->init(cfg)) {
    LOGERR("Error in _urScriptBuilder->init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initStateFileHandler(
  const std::string &filePath, UrControlBloom &bloom) {
  if (ErrorCode::SUCCESS != bloom._stateFileHandler->init(filePath)) {
    LOGERR("Error in _stateFileHandler->init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlBloomInitHelper::initMotionExecutor(
  const UrControlBloomMotionSequenceConfig &cfg, UrControlBloom &bloom) {
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

  auto bloomMotionSequence = std::make_unique<BloomMotionSequence>(
    cfg.bloomMotionSequenceCfg, Motion::BLOOM_MOTION_SEQUENCE_NAME, 
    Motion::BLOOM_MOTION_ID, bloom._urScriptBuilder, bloom._stateFileHandler);
  if (ErrorCode::SUCCESS != 
      bloom._motionExecutor.addSequence(std::move(bloomMotionSequence))) {
    LOGERR("Error in motionExecutor.addSequence() for BloomMotionSequence");
    return ErrorCode::FAILURE;
  }

  auto jengaMotionSequence = std::make_unique<JengaMotionSequence>(
    cfg.jengaMotionSequenceCfg, Motion::JENGA_MOTION_SEQUENCE_NAME, 
    Motion::JENGA_MOTION_ID, bloom._urScriptBuilder, bloom._stateFileHandler);
  if (ErrorCode::SUCCESS != 
      bloom._motionExecutor.addSequence(std::move(jengaMotionSequence))) {
    LOGERR("Error in motionExecutor.addSequence() for JengaMotionSequence");
    return ErrorCode::FAILURE;
  }

  auto parkMotionSequence = std::make_unique<ParkMotionSequence>(
    cfg.parkMotionSequenceCfg, Motion::PARK_MOTION_SEQUENCE_NAME, 
    Motion::PARK_ID, bloom._urScriptBuilder, bloom._stateFileHandler);
  if (ErrorCode::SUCCESS != 
      bloom._motionExecutor.addSequence(std::move(parkMotionSequence))) {
    LOGERR("Error in motionExecutor.addSequence() for ParkMotionSequence");
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
  state.handleEvent = nullptr;
  stateDescriptions.push_back(state);

  state.name = BloomState::IDLE;
  state.onEnter = std::bind(&UrControlBloom::enterIdleState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitIdleState, &bloom);
  state.handleEvent = 
    std::bind(&UrControlBloom::handleEventIdleState, &bloom, _1);
  stateDescriptions.push_back(state);

  state.name = BloomState::PARK;
  state.onEnter = std::bind(&UrControlBloom::enterParkState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitParkState, &bloom);
  state.handleEvent = 
    std::bind(&UrControlBloom::handleEventParkState, &bloom, _1);
  stateDescriptions.push_back(state);

  state.name = BloomState::BLOOM;
  state.onEnter = std::bind(&UrControlBloom::enterBloomState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitBloomState, &bloom);
  state.handleEvent = 
    std::bind(&UrControlBloom::handleEventBloomState, &bloom, _1);
  stateDescriptions.push_back(state);

  state.name = BloomState::BLOOM_RECOVERY;
  state.onEnter = std::bind(&UrControlBloom::enterBloomRecoveryState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitBloomRecoveryState, &bloom);
  state.handleEvent = 
    std::bind(&UrControlBloom::handleEventBloomRecoveryState, &bloom, _1);
  stateDescriptions.push_back(state);

  state.name = BloomState::JENGA;
  state.onEnter = std::bind(&UrControlBloom::enterJengaState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitJengaState, &bloom);
  state.handleEvent = 
    std::bind(&UrControlBloom::handleEventJengaState, &bloom, _1);
  stateDescriptions.push_back(state);

  state.name = BloomState::JENGA_RECOVERY;
  state.onEnter = std::bind(&UrControlBloom::enterJengaRecoveryState, &bloom);
  state.onExit = std::bind(&UrControlBloom::exitJengaRecoveryState, &bloom);
  state.handleEvent = 
    std::bind(&UrControlBloom::handleEventJengaRecoveryState, &bloom, _1);
  stateDescriptions.push_back(state);

  std::vector<StateTransitions> stateTransitions;
  StateTransitions transition;
  transition.stateName = BloomState::INIT;
  transition.transitions.insert(BloomState::BLOOM_RECOVERY);
  transition.transitions.insert(BloomState::JENGA_RECOVERY);
  transition.transitions.insert(BloomState::PARK);
  transition.transitions.insert(BloomState::IDLE);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::IDLE;
  transition.transitions.insert(BloomState::BLOOM_RECOVERY);
  transition.transitions.insert(BloomState::JENGA_RECOVERY);
  transition.transitions.insert(BloomState::PARK);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::PARK;
  transition.transitions.insert(BloomState::IDLE);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::BLOOM_RECOVERY;
  transition.transitions.insert(BloomState::BLOOM);

  //a potential recovery scenario with BloonEndStrategy::TRANSPORT_AND_WAIT
  transition.transitions.insert(BloomState::IDLE);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::JENGA_RECOVERY;
  transition.transitions.insert(BloomState::JENGA);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::BLOOM;
  transition.transitions.insert(BloomState::JENGA);
  transition.transitions.insert(BloomState::IDLE);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  transition.stateName = BloomState::JENGA;
  transition.transitions.insert(BloomState::BLOOM_RECOVERY);
  transition.transitions.insert(BloomState::IDLE);
  stateTransitions.push_back(transition);
  transition.transitions.clear();

  if (ErrorCode::SUCCESS != sm.init(
    StateLogging::ENABLED, stateDescriptions, stateTransitions)) {
    LOGERR("Error in stateMachine.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}
