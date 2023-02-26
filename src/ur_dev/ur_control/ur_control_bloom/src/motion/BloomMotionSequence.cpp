//Corresponding header
#include "ur_control_bloom/motion/BloomMotionSequence.h"

//System headers

//Other libraries headers
#include "urscript_common/motion/MotionUtils.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

BloomMotionSequence::BloomMotionSequence(
  const BloomMotionSequenceConfig& cfg, const std::string& name, int32_t id,
  const std::shared_ptr<UrScriptBuilder>& urScriptBuilder,
  const std::shared_ptr<StateFileHandler>& stateFileHandler) : 
    MotionSequence(name, id, urScriptBuilder), _cfg(cfg), 
    _stateFileHandler(stateFileHandler) {
  loadState();
}

void BloomMotionSequence::start(const UrscriptsBatchDoneCb& cb) {
  const std::vector<UrscriptCommand> commands {
    generateGraspCommand(), 
    generateTransportAndPlaceCommand(), 
    generateRetractAndReturnHomeCommand()
  };

  dispatchUscriptsAsyncCb(commands, cb);
}

void BloomMotionSequence::gracefulStop(const UrscriptsBatchDoneCb& cb) {
  //for now the graceful_stop and recover implementations are identical
  recover(cb);
}

void BloomMotionSequence::recover(const UrscriptsBatchDoneCb& cb) {
  std::vector<UrscriptCommand> commands { generateReturnHomeCommand() };
  if (_state.holdingObject) {
    commands.push_back(generateTransportAndPlaceCommand());
    commands.push_back(generateRetractAndReturnHomeCommand());
  }
  commands.push_back(generateReturnHomeAndOpenGripperCommand());

  dispatchUscriptsAsyncCb(commands, cb);
}

UrscriptCommand BloomMotionSequence::generateGraspCommand() {
  //NOTE: even though MoveJointCommand waypoints are used
  //their respective Cartesian counterparts are used to compute blending radius
  const double blendingRadius = computeSafeBlendingRadius(
    _cfg.homeCartesian.pos, _cfg.graspApproachCartesian.pos, 
    _cfg.graspCartesian.pos);

  auto graspApproachCommand = std::make_unique<MoveJointCommand>(
    _cfg.graspApproachJoint, _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc, 
    blendingRadius);
  auto graspCommand = std::make_unique<MoveJointCommand>(_cfg.graspJoint);
  constexpr int32_t gripperSpeedPercent = 100;
  auto gripperSpeedCommand = std::make_unique<GripperParamCommand>(
    GripperParamType::SPEED, gripperSpeedPercent);
  constexpr int32_t gripperForcePercent = 50;
  auto gripperForceCommand = std::make_unique<GripperParamCommand>(
    GripperParamType::FORCE, gripperForcePercent);
  auto closeGripperCommand = 
    std::make_unique<GripperActuateCommand>(GripperActuateType::CLOSE);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(graspApproachCommand))
              .addCommand(std::move(graspCommand))
              .addCommand(std::move(gripperSpeedCommand))
              .addCommand(std::move(gripperForceCommand))
              .addCommand(std::move(closeGripperCommand));
  const UrScriptPayload cmdPayload = 
    constructUrScript(Motion::Bloom::GRASP_NAME, cmdContainer);

  const UrscriptDoneCb doneCb = [this](){
    _state.holdingObject = true;
    serializeState();
  };
  return { cmdPayload, doneCb };
}

UrscriptCommand BloomMotionSequence::generateTransportAndPlaceCommand() {
  UrScriptCommandContainer cmdContainer;
  TransportMoveCommands transportMoveCommands = 
    generateTransportMoveCommands(TransportStrategy::TWIST);
  for (auto& moveCommand : transportMoveCommands) {
    cmdContainer.addCommand(std::move(moveCommand));
  }

  if (BloomEndStrategy::PLACE_AND_RETURN_HOME == _cfg.endStrategy) {
    auto placeCommand = 
      std::make_unique<MoveLinearCommand>(_cfg.placeCartesian);
      constexpr int32_t gripperSpeedPercent = 100;
    auto gripperSpeedCommand = std::make_unique<GripperParamCommand>(
      GripperParamType::SPEED, gripperSpeedPercent);
    constexpr int32_t gripperForcePercent = 50;
    auto gripperForceCommand = std::make_unique<GripperParamCommand>(
      GripperParamType::FORCE, gripperForcePercent);
    auto openGripperCommand = 
      std::make_unique<GripperActuateCommand>(GripperActuateType::OPEN);

    cmdContainer.addCommand(std::move(gripperSpeedCommand))
                .addCommand(std::move(gripperForceCommand))
                .addCommand(std::move(openGripperCommand))
                .addCommand(std::move(placeCommand));
  }

  const UrScriptPayload cmdPayload = 
    constructUrScript(Motion::Bloom::TRANSPORT_AND_PLACE_NAME, cmdContainer);
  const UrscriptDoneCb doneCb = [this](){
    _state.holdingObject = false;
    serializeState();
  };
  return { cmdPayload, doneCb };
}

UrscriptCommand BloomMotionSequence::generateRetractAndReturnHomeCommand() {
  //NOTE: manually restrict the level of blending radius
  //Even though a bigger blending radius could be achieved, this might result 
  //in collision with the container that the rose will be placed into
  constexpr double blendingRadius = 0.10; //[m]
  auto placeRetractCommand = std::make_unique<MoveLinearCommand>(
    _cfg.placeApproachCartesian, _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc, 
    blendingRadius);
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(placeRetractCommand))
              .addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload = constructUrScript(
    Motion::Bloom::RETRACT_AND_RETURN_HOME_NAME, cmdContainer);

  return { cmdPayload };
}

UrscriptCommand BloomMotionSequence::generateReturnHomeCommand() {
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload =
    constructUrScript(Motion::Bloom::RETURN_HOME_NAME, cmdContainer);

  return { cmdPayload };
}

UrscriptCommand BloomMotionSequence::generateReturnHomeAndOpenGripperCommand() {
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
  auto openGripperCommand = 
    std::make_unique<GripperActuateCommand>(
      GripperActuateType::OPEN, GripperCommandPolicy::NON_BLOCKING);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(returnHomeCommand))
              .addCommand(std::move(openGripperCommand));
  const UrScriptPayload cmdPayload = constructUrScript(
    Motion::Bloom::RETURN_HOME_AND_OPEN_GRIPPER_NAME, cmdContainer);

  return { cmdPayload };
}

TransportMoveCommands BloomMotionSequence::generateTransportMoveCommands(
  TransportStrategy strategy) const {
  switch (strategy) {
  case TransportStrategy::BASIC:
    return generateBasicTransportMoveCommands();

  case TransportStrategy::FULL_ROTATION:
    return generateFullRotationTransportMoveCommands();

  case TransportStrategy::TWIST:
    return generateTwistTransportMoveCommands();
  
  default:
    LOGERR("Error, received unsupported TransportStrategy: [%d]", 
           getEnumValue(strategy));
    break;
  }
}

TransportMoveCommands 
BloomMotionSequence::generateBasicTransportMoveCommands() const {
  TransportMoveCommands cmds;
  double blendingRadius = 0.20; //[m]
  auto graspApproachCommand = std::make_unique<MoveJointCommand>(
    _cfg.graspApproachJoint, _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc, 
    blendingRadius);

  if (BloomEndStrategy::WAIT_AFTER_TRANSPORT == _cfg.endStrategy) {
    //for this strategy this will be the last point, thus apply zero blending
    blendingRadius = 0.0;
  }

  auto placeApproachCommand = std::make_unique<MoveJointCommand>(
    _cfg.placeApproachBasicStrategyJoint, _cfg.pickAndPlaceVel, 
    _cfg.pickAndPlaceAcc, blendingRadius);

  cmds.push_back(std::move(graspApproachCommand));
  cmds.push_back(std::move(placeApproachCommand));
  return cmds;
}

TransportMoveCommands 
BloomMotionSequence::generateFullRotationTransportMoveCommands() const {
  TransportMoveCommands cmds;
  double blendingRadius = 0.20; //[m]
  auto graspApproachCommand = std::make_unique<MoveJointCommand>(
    _cfg.graspApproachJoint, _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc, 
    blendingRadius);

  if (BloomEndStrategy::WAIT_AFTER_TRANSPORT == _cfg.endStrategy) {
    //for this strategy this will be the last point, thus apply zero blending
    blendingRadius = 0.0;
  }

  auto placeApproachCommand = std::make_unique<MoveJointCommand>(
    _cfg.placeApproachFullRotationStrategyJoint, _cfg.pickAndPlaceVel, 
    _cfg.pickAndPlaceAcc, blendingRadius);
    
  cmds.push_back(std::move(graspApproachCommand));
  cmds.push_back(std::move(placeApproachCommand));
  return cmds;
}

TransportMoveCommands 
BloomMotionSequence::generateTwistTransportMoveCommands() const {
  TransportMoveCommands cmds;
  double blendingRadius = 0.30; //[m]
  //NOTE: 
  //graspApproachJoint is accounted for in the twistStrategyWaypointOneJoint
  auto twistStrategyWaypointOneJointCommand = 
    std::make_unique<MoveJointCommand>(_cfg.twistStrategyWaypointOneJoint, 
    _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc, blendingRadius);

  auto twistStrategyWaypointTwoJointCommand = 
    std::make_unique<MoveJointCommand>(_cfg.twistStrategyWaypointTwoJoint, 
    _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc, blendingRadius);

  if (BloomEndStrategy::PLACE_AND_RETURN_HOME == _cfg.endStrategy) {
    //reduce the blending to prevent collision with the placement container
    blendingRadius = 0.20; //[m]
  } else { //BloomEndStrategy::WAIT_AFTER_TRANSPORT
    //for this strategy this will be the last point, thus apply zero blending
    blendingRadius = 0.0; //[m]
  }

  auto twistStrategyWaypointThreeJointCommand = 
    std::make_unique<MoveJointCommand>(_cfg.twistStrategyWaypointThreeJoint, 
    _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc, blendingRadius);
    
  cmds.push_back(std::move(twistStrategyWaypointOneJointCommand));
  cmds.push_back(std::move(twistStrategyWaypointTwoJointCommand));
  cmds.push_back(std::move(twistStrategyWaypointThreeJointCommand));
  return cmds;
}

void BloomMotionSequence::loadState() {
  _state.holdingObject = [this](){
    std::string strValue;
    const ErrorCode errCode = _stateFileHandler->getEntry(
      Motion::Bloom::SECTION_NAME, Motion::Bloom::HOLDING_OBJECT_ENTRY_NAME, 
      strValue);
    if (ErrorCode::SUCCESS != errCode) {
      LOGERR("Error trying to getEntry(): [%s] for section: [%s]. "
            "Defaulting to False", Motion::Bloom::HOLDING_OBJECT_ENTRY_NAME, 
            Motion::Bloom::SECTION_NAME);
      return false;
    }

    if (BOOL_TRUE_VALUE_STR == strValue) {
      return true;
    }
    return false;
  }();
}

void BloomMotionSequence::serializeState() {
  const std::string holdingObjStr = _state.holdingObject ? 
    BOOL_TRUE_VALUE_STR : BOOL_FALSE_VALUE_STR;
  const ErrorCode errCode = _stateFileHandler->updateEntry(
    Motion::Bloom::SECTION_NAME, Motion::Bloom::HOLDING_OBJECT_ENTRY_NAME, 
    holdingObjStr);
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error trying to serialize BloomMotionSequenceState");
  }
}
