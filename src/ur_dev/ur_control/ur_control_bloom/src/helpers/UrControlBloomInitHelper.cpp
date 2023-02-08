//Corresponding header
#include "ur_control_bloom/helpers/UrControlBloomInitHelper.h"

//System headers

//Other libraries headers
#include "urscript_common/defines/UrScriptStructs.h"
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
  //TODO parse from files
  const AngleAxis orientation(0.0, -3.16, 0.0);
  constexpr double accel = 1.0;
  constexpr double vel = 1.0;
  constexpr double blendingRadius = 0.0;

  const WaypointCartesian graspPose(Point3d(-0.5, -0.4, 0.2), orientation);
  const MoveLinearCommand graspCommand(graspPose, accel, vel, blendingRadius);

  const WaypointCartesian transportPose(Point3d(0.0, -0.4, 0.2), orientation);
  const MoveLinearCommand transportCommand(
      transportPose, accel, vel, blendingRadius);

  const WaypointCartesian homePose(Point3d(0.5, -0.4, 0.6), orientation);
  const MoveLinearCommand returnHomeCommand(
      homePose, accel, vel, blendingRadius);

  UrScriptHeaders headers;
  auto& graspHeaderValue = headers[Motion::Bloom::GRASP_NAME];
  graspHeaderValue = "def BloomGrasp():\n\t";
  graspHeaderValue.append(graspCommand.serialize()).append("\nend\n");
  LOGC("%s", graspHeaderValue.c_str());

  auto& transportAndPlaceHeaderValue = 
    headers[Motion::Bloom::TRANSPORT_AND_PLACE_NAME];
  transportAndPlaceHeaderValue = "def BloomTransportAndPlace():\n\t";
  transportAndPlaceHeaderValue.append(
      transportCommand.serialize()).append("\nend\n");
  LOGC("%s", transportAndPlaceHeaderValue.c_str());

  auto& returnHomeHeaderValue = headers[Motion::Bloom::RETURN_HOME_NAME];
  returnHomeHeaderValue = "def BloomReturnHome():\n\t";
  returnHomeHeaderValue.append(
      returnHomeCommand.serialize()).append("\nend\n");
  LOGC("%s", returnHomeHeaderValue.c_str());

  auto bloomMotionSequence = std::make_unique<BloomMotionSequence>(
    Motion::BLOOM_MOTION_SEQUENCE_NAME, Motion::BLOOM_MOTION_ID, 
    std::move(headers));

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
  //TODO parse from files
  const AngleAxis orientation(0.0, -3.16, 0.0);
  constexpr double accel = 1.0;
  constexpr double vel = 1.0;
  constexpr double blendingRadius = 0.0;

  const WaypointCartesian graspPose(Point3d(-0.5, -0.6, 0.2), orientation);
  const MoveLinearCommand graspCommand(graspPose, accel, vel, blendingRadius);

  const WaypointCartesian transportPose(Point3d(0.0, -0.6, 0.2), orientation);
  const MoveLinearCommand transportCommand(
      transportPose, accel, vel, blendingRadius);

  const WaypointCartesian homePose(Point3d(0.5, -0.4, 0.6), orientation);
  const MoveLinearCommand returnHomeCommand(
      homePose, accel, vel, blendingRadius);

  UrScriptHeaders headers;
  auto& graspHeaderValue = headers[Motion::Jenga::GRASP_NAME];
  graspHeaderValue = "def JengaGrasp():\n\t";
  graspHeaderValue.append(graspCommand.serialize()).append("\nend\n");

  auto& transportAndPlaceHeaderValue = 
    headers[Motion::Jenga::TRANSPORT_AND_PLACE_NAME];
  transportAndPlaceHeaderValue = "def JengaTransportAndPlace():\n\t";
  transportAndPlaceHeaderValue.append(
      transportCommand.serialize()).append("\nend\n");

  auto& returnHomeHeaderValue = headers[Motion::Jenga::RETURN_HOME_NAME];
  returnHomeHeaderValue = "def JengaReturnHome():\n\t";
  returnHomeHeaderValue.append(
      returnHomeCommand.serialize()).append("\nend\n");

  auto jengaMotionSequence = std::make_unique<JengaMotionSequence>(
    Motion::JENGA_MOTION_SEQUENCE_NAME, Motion::JENGA_MOTION_ID, 
    std::move(headers));

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