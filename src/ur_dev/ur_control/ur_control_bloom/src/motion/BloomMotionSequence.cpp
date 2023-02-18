//Corresponding header
#include "ur_control_bloom/motion/BloomMotionSequence.h"

//System headers

//Other libraries headers
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
  const std::vector<UscriptCommand> commands {
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
  std::vector<UscriptCommand> commands;
  if (_state.holdingObject) {
    commands.push_back(generateTransportAndPlaceCommand());
    commands.push_back(generateRetractAndReturnHomeCommand());
  }
  commands.push_back(generateReturnHomeAndOpenGripperCommand());

  dispatchUscriptsAsyncCb(commands, cb);
}

UscriptCommand BloomMotionSequence::generateGraspCommand() {
  auto graspApproachCommand = 
    std::make_unique<MoveJointCommand>(_cfg.graspApproachJoint);
  auto graspCommand = std::make_unique<MoveJointCommand>(_cfg.graspJoint);
  auto closeGripperCommand = 
    std::make_unique<GripperActuateCommand>(GripperActuateType::CLOSE);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(graspApproachCommand))
              .addCommand(std::move(graspCommand))
              .addCommand(std::move(closeGripperCommand));
  const UrScriptPayload cmdPayload = 
    constructUrScript(Motion::Bloom::GRASP_NAME, cmdContainer);

  const UrscriptDoneCb doneCb = [this](){
    _state.holdingObject = true;
    serializeState();
  };
  return { cmdPayload, doneCb };
}

UscriptCommand BloomMotionSequence::generateTransportAndPlaceCommand() {
  auto placeApproachCommand = 
  std::make_unique<MoveJointCommand>(_cfg.placeApproachJoint);
  auto placeCommand = std::make_unique<MoveLinearCommand>(_cfg.placeCartesian);
  auto openGripperCommand = 
    std::make_unique<GripperActuateCommand>(GripperActuateType::OPEN);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(placeApproachCommand))
              .addCommand(std::move(placeCommand))
              .addCommand(std::move(openGripperCommand));
  const UrScriptPayload cmdPayload = 
    constructUrScript(Motion::Bloom::TRANSPORT_AND_PLACE_NAME, cmdContainer);

  const UrscriptDoneCb doneCb = [this](){
    _state.holdingObject = false;
    serializeState();
  };
  return { cmdPayload, doneCb };
}

UscriptCommand BloomMotionSequence::generateRetractAndReturnHomeCommand() {
  auto placeRetractCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.placeApproachCartesian);
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(placeRetractCommand))
              .addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload = constructUrScript(
    Motion::Bloom::RETRACT_AND_RETURN_HOME_NAME, cmdContainer);

  return { cmdPayload };
}

UscriptCommand BloomMotionSequence::generateReturnHomeCommand() {
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload =
    constructUrScript(Motion::Bloom::RETURN_HOME_NAME, cmdContainer);

  return { cmdPayload };
}

UscriptCommand BloomMotionSequence::generateReturnHomeAndOpenGripperCommand() {
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
