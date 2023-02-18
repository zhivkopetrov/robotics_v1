//Corresponding header
#include "ur_control_bloom/motion/JengaMotionSequence.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

namespace {
constexpr auto TOWER_DIR_A_TO_B_STR = "A_TO_B";
constexpr auto TOWER_DIR_B_TO_A_STR = "B_TO_A";
}

JengaMotionSequence::JengaMotionSequence(
  const JengaMotionSequenceConfig& cfg, const std::string& name, int32_t id, 
  const std::shared_ptr<UrScriptBuilder>& urScriptBuilder,
  const std::shared_ptr<StateFileHandler>& stateFileHandler) 
  : MotionSequence(name, id, urScriptBuilder), _cfg(cfg),
  _stateFileHandler(stateFileHandler) {
  loadState();
}

void JengaMotionSequence::start(const UscriptsBatchDoneCb& cb) {
  const std::vector<UscriptCommand> commands {
    generateGraspCommand(), 
    generateTransportAndPlaceCommand(), 
    generateReturnHomeCommand()
  };

  dispatchUscriptsAsyncCb(commands, cb);
}

void JengaMotionSequence::gracefulStop(const UscriptsBatchDoneCb& cb) {
  //for now the graceful_stop and recover implementations are identical
  recover(cb);
}

void JengaMotionSequence::recover(const UscriptsBatchDoneCb& cb) {
  std::vector<UscriptCommand> commands;
  if (_state.holdingObject) {
    commands.push_back(generateTransportAndPlaceCommand());
  }
  commands.push_back(generateReturnHomeAndOpenGripperCommand());

  dispatchUscriptsAsyncCb(commands, cb);
}

UscriptCommand JengaMotionSequence::generateGraspCommand() {
  auto graspApproachCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.graspApproachCartesian);
  auto baseCenterACommand = 
    std::make_unique<MoveLinearCommand>(_cfg.baseCenterACartesian);
  auto closeGripperCommand = 
    std::make_unique<GripperActuateCommand>(GripperActuateType::CLOSE);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(graspApproachCommand))
              .addCommand(std::move(baseCenterACommand))
              .addCommand(std::move(closeGripperCommand));
  const UrScriptPayload cmdPayload = 
    constructUrScript(Motion::Jenga::GRASP_NAME, cmdContainer);

  const UscriptDoneCb doneCb = [this](){
    _state.holdingObject = true;
    serializeState();
  };
  return { cmdPayload, doneCb };
}

UscriptCommand JengaMotionSequence::generateTransportAndPlaceCommand() {
  auto transportApproachCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.graspApproachCartesian);
  auto baseCenterBCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.baseCenterBCartesian);
  auto openGripperCommand = 
    std::make_unique<GripperActuateCommand>(GripperActuateType::OPEN);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(transportApproachCommand))
              .addCommand(std::move(baseCenterBCommand))
              .addCommand(std::move(openGripperCommand));
  const UrScriptPayload cmdPayload = constructUrScript(
    Motion::Jenga::TRANSPORT_AND_PLACE_NAME, cmdContainer);

  const UscriptDoneCb doneCb = [this](){
    _state.holdingObject = false;
    serializeState();
  };
  return { cmdPayload, doneCb };
}

UscriptCommand JengaMotionSequence::generateReturnHomeCommand() {
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload = 
    constructUrScript(Motion::Jenga::RETURN_HOME_NAME, cmdContainer);

  return { cmdPayload };
}

UscriptCommand JengaMotionSequence::generateReturnHomeAndOpenGripperCommand() {
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
  auto openGripperCommand = std::make_unique<GripperActuateCommand>(
    GripperActuateType::OPEN, GripperCommandPolicy::NON_BLOCKING);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(openGripperCommand))
              .addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload = constructUrScript(
    Motion::Jenga::RETURN_HOME_AND_OPEN_GRIPPER_NAME, cmdContainer);

  return { cmdPayload };
}

void JengaMotionSequence::loadState() {
  _state.holdingObject = [this](){
    std::string strValue;
    const ErrorCode errCode = _stateFileHandler->getEntry(
      Motion::Jenga::SECTION_NAME, Motion::Jenga::HOLDING_OBJECT_ENTRY_NAME, 
      strValue);
    if (ErrorCode::SUCCESS != errCode) {
      LOGERR("Error trying to getEntry(): [%s] for section: [%s]. "
            "Defaulting to [%s]", Motion::Jenga::HOLDING_OBJECT_ENTRY_NAME, 
            Motion::Jenga::SECTION_NAME, BOOL_FALSE_VALUE_STR);
      return false;
    }

    if (BOOL_TRUE_VALUE_STR == strValue) {
      return true;
    }
    return false;
  }();

  _state.towerDirection = [this](){
    constexpr auto defaultDir = TowerDirection::A_TO_B;
    std::string strValue;
    const ErrorCode errCode = _stateFileHandler->getEntry(
      Motion::Jenga::SECTION_NAME, Motion::Jenga::DIRECTION_ENTRY_NAME, 
      strValue);
    if (ErrorCode::SUCCESS != errCode) {
      LOGERR("Error trying to getEntry(): [%s] for section: [%s]. "
            "Defaulting to [%s]", Motion::Jenga::DIRECTION_ENTRY_NAME, 
            Motion::Jenga::SECTION_NAME, TOWER_DIR_A_TO_B_STR);
      return defaultDir;
    }

    if (TOWER_DIR_A_TO_B_STR == strValue) {
      return TowerDirection::A_TO_B;
    }
    return TowerDirection::B_TO_A;
  }();

  _state.currentObjectIdx = [this](){
    constexpr auto defaultCurrObjIdx = 0;
    std::string strValue;
    const ErrorCode errCode = _stateFileHandler->getEntry(
      Motion::Jenga::SECTION_NAME, Motion::Jenga::CURRENT_OBJECT_IDX_ENTRY_NAME, 
      strValue);
    if (ErrorCode::SUCCESS != errCode) {
      LOGERR("Error trying to getEntry(): [%s] for section: [%s]. "
            "Defaulting to [%d]", Motion::Jenga::CURRENT_OBJECT_IDX_ENTRY_NAME, 
            Motion::Jenga::SECTION_NAME, defaultCurrObjIdx);
      return defaultCurrObjIdx;
    }

    try {
      return std::stoi(strValue);
    } catch (const std::exception &e) {
      LOGERR("%s", e.what());
      return defaultCurrObjIdx;
    }
  }();
}

void JengaMotionSequence::serializeState() {
  const std::string holdingObjStr = _state.holdingObject ? 
    BOOL_TRUE_VALUE_STR : BOOL_FALSE_VALUE_STR;
  ErrorCode errCode = _stateFileHandler->updateEntry(
    Motion::Jenga::SECTION_NAME, Motion::Jenga::HOLDING_OBJECT_ENTRY_NAME, 
    holdingObjStr);
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error trying to serialize JengaMotionSequenceState");
  }

  errCode = _stateFileHandler->updateEntry(
    Motion::Jenga::SECTION_NAME, Motion::Jenga::CURRENT_OBJECT_IDX_ENTRY_NAME, 
    std::to_string(_state.currentObjectIdx));
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error trying to serialize JengaMotionSequenceState");
  }

  const std::string directionStr = 
    TowerDirection::A_TO_B == _state.towerDirection ? 
      TOWER_DIR_A_TO_B_STR : TOWER_DIR_B_TO_A_STR;
  errCode = _stateFileHandler->updateEntry(
    Motion::Jenga::SECTION_NAME, Motion::Jenga::DIRECTION_ENTRY_NAME, 
    directionStr);
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error trying to serialize JengaMotionSequenceState");
  }
}
