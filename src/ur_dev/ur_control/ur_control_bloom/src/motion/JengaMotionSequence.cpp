//Corresponding header
#include "ur_control_bloom/motion/JengaMotionSequence.h"

//System headers

//Other libraries headers
#include "urscript_common/urscript/UrScriptBuilder.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

JengaMotionSequence::JengaMotionSequence(
  const JengaMotionSequenceConfig& cfg,
  const std::string& name, int32_t id) : MotionSequence(name, id), _cfg(cfg) {

}

void JengaMotionSequence::start(const UscriptsBatchDoneCb& cb) {
  std::vector<UscriptCommand> commands;
  UrscriptCommandContainer cmdContainer;
  UrScriptPayload cmdPayload;

  auto graspApproachCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.graspApproachCartesian);
  auto baseCenterACommand = 
    std::make_unique<MoveLinearCommand>(_cfg.baseCenterACartesian);
  cmdContainer.addCommand(std::move(graspApproachCommand))
              .addCommand(std::move(baseCenterACommand));
  cmdPayload = UrScriptBuilder::construct(
    Motion::Jenga::GRASP_NAME, cmdContainer);
  commands.push_back( { cmdPayload } );

  auto transportApproachCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.graspApproachCartesian);
  auto baseCenterBCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.baseCenterBCartesian);

  cmdContainer.addCommand(std::move(transportApproachCommand))
              .addCommand(std::move(baseCenterBCommand));
  cmdPayload = UrScriptBuilder::construct(
    Motion::Jenga::TRANSPORT_AND_PLACE_NAME, cmdContainer);
  commands.push_back( { cmdPayload } );

  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
  cmdContainer.addCommand(std::move(returnHomeCommand));
  cmdPayload = UrScriptBuilder::construct(
    Motion::Jenga::RETURN_HOME_NAME, cmdContainer);
  commands.push_back( { cmdPayload } );

  dispatchUscriptsAsyncCb(commands, cb);
}

void JengaMotionSequence::gracefulStop(const UscriptsBatchDoneCb& cb) {
  UrscriptCommandContainer cmdContainer;

  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
  cmdContainer.addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload = 
    UrScriptBuilder::construct(Motion::Jenga::RETURN_HOME_NAME, cmdContainer);

  const std::vector<UscriptCommand> commands {
    { cmdPayload }
  };

  dispatchUscriptsAsyncCb(commands, cb);
}

void JengaMotionSequence::recover(const UscriptsBatchDoneCb& cb) {
  std::vector<UscriptCommand> commands;
  UrscriptCommandContainer cmdContainer;

  //TODO fill on init
  constexpr bool holdingJenga = false;
  if (holdingJenga) {
    auto placeApproachCommand = 
      std::make_unique<MoveJointCommand>(_cfg.graspApproachJoint);

    //TODO recover from file where should jenga be placed
    auto placeCommand = 
      std::make_unique<MoveLinearCommand>(_cfg.baseCenterACartesian);
    cmdContainer.addCommand(std::move(placeApproachCommand))
                .addCommand(std::move(placeCommand));
    const UrScriptPayload cmdPayload = UrScriptBuilder::construct(
      Motion::Jenga::TRANSPORT_AND_PLACE_NAME, cmdContainer);
    commands.push_back( { cmdPayload } );
  } else {
    auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
    cmdContainer.addCommand(std::move(returnHomeCommand));
    const UrScriptPayload cmdPayload = 
      UrScriptBuilder::construct(Motion::Jenga::RETURN_HOME_NAME, cmdContainer);
    commands.push_back( { cmdPayload } );
  }

  dispatchUscriptsAsyncCb(commands, cb);
}