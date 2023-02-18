//Corresponding header
#include "ur_control_bloom/motion/JengaMotionSequence.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

JengaMotionSequence::JengaMotionSequence(
  const JengaMotionSequenceConfig& cfg, const std::string& name, int32_t id, 
  const std::shared_ptr<UrScriptBuilder>& urScriptBuilder,
  const std::shared_ptr<StateFileHandler>& stateFileHandler) 
  : MotionSequence(name, id, urScriptBuilder), _cfg(cfg),
  _stateFileHandler(stateFileHandler) {
  loadState();
}

void JengaMotionSequence::start(const UscriptsBatchDoneCb& cb) {
  std::vector<UscriptCommand> commands;
  UrScriptCommandContainer cmdContainer;
  UrScriptPayload cmdPayload;

  auto graspApproachCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.graspApproachCartesian);
  auto baseCenterACommand = 
    std::make_unique<MoveLinearCommand>(_cfg.baseCenterACartesian);
  auto closeGripperCommand = 
    std::make_unique<GripperActuateCommand>(GripperActuateType::CLOSE);
  cmdContainer.addCommand(std::move(graspApproachCommand))
              .addCommand(std::move(baseCenterACommand))
              .addCommand(std::move(closeGripperCommand));
  cmdPayload = constructUrScript(Motion::Jenga::GRASP_NAME, cmdContainer);
  commands.push_back( { cmdPayload } );

  auto transportApproachCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.graspApproachCartesian);
  auto baseCenterBCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.baseCenterBCartesian);
  auto openGripperCommand = 
    std::make_unique<GripperActuateCommand>(GripperActuateType::OPEN);

  cmdContainer.addCommand(std::move(transportApproachCommand))
              .addCommand(std::move(baseCenterBCommand))
              .addCommand(std::move(openGripperCommand));
  cmdPayload = constructUrScript(
    Motion::Jenga::TRANSPORT_AND_PLACE_NAME, cmdContainer);
  commands.push_back( { cmdPayload } );

  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
  cmdContainer.addCommand(std::move(returnHomeCommand));
  cmdPayload = constructUrScript(Motion::Jenga::RETURN_HOME_NAME, cmdContainer);
  commands.push_back( { cmdPayload } );

  dispatchUscriptsAsyncCb(commands, cb);
}

void JengaMotionSequence::gracefulStop(const UscriptsBatchDoneCb& cb) {
  UrScriptCommandContainer cmdContainer;

  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
  cmdContainer.addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload = 
    constructUrScript(Motion::Jenga::RETURN_HOME_NAME, cmdContainer);

  const std::vector<UscriptCommand> commands {
    { cmdPayload }
  };

  dispatchUscriptsAsyncCb(commands, cb);
}

void JengaMotionSequence::recover(const UscriptsBatchDoneCb& cb) {
  std::vector<UscriptCommand> commands;
  UrScriptCommandContainer cmdContainer;

  //TODO fill on init
  constexpr bool holdingJenga = false;
  if (holdingJenga) {
    auto placeApproachCommand = 
      std::make_unique<MoveJointCommand>(_cfg.graspApproachJoint);

    //TODO recover from file where should jenga be placed
    auto placeCommand = 
      std::make_unique<MoveLinearCommand>(_cfg.baseCenterACartesian);
    auto openGripperCommand = 
      std::make_unique<GripperActuateCommand>(GripperActuateType::OPEN);

    cmdContainer.addCommand(std::move(placeApproachCommand))
                .addCommand(std::move(placeCommand))
                .addCommand(std::move(openGripperCommand));
    const UrScriptPayload cmdPayload = 
      constructUrScript(Motion::Jenga::TRANSPORT_AND_PLACE_NAME, cmdContainer);
    commands.push_back( { cmdPayload } );
  } else {
    auto openGripperCommand = 
      std::make_unique<GripperActuateCommand>(
        GripperActuateType::OPEN, GripperCommandPolicy::NON_BLOCKING);
    auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
    cmdContainer.addCommand(std::move(openGripperCommand))
                .addCommand(std::move(returnHomeCommand));
    const UrScriptPayload cmdPayload = 
      constructUrScript(Motion::Jenga::RETURN_HOME_NAME, cmdContainer);
    commands.push_back( { cmdPayload } );
  }

  dispatchUscriptsAsyncCb(commands, cb);
}

void JengaMotionSequence::loadState() {
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

void JengaMotionSequence::serializeState() {
  const std::string holdingObjStr = _state.holdingObject ? "True" : "False";
  const ErrorCode errCode = _stateFileHandler->updateEntry(
    Motion::Jenga::SECTION_NAME, Motion::Jenga::HOLDING_OBJECT_ENTRY_NAME, 
    holdingObjStr);
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error trying to serialize JengaMotionSequenceState");
  }
}
