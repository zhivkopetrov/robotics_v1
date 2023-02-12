//Corresponding header
#include "ur_control_bloom/motion/BloomMotionSequence.h"

//System headers

//Other libraries headers
#include "urscript_common/urscript/UrScriptBuilder.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

BloomMotionSequence::BloomMotionSequence(
  const BloomMotionSequenceConfig& cfg, const std::string& name, int32_t id) : 
    MotionSequence(name, id), _cfg(cfg) {
      
}

void BloomMotionSequence::start(const UscriptsBatchDoneCb& cb) {
  std::vector<UscriptCommand> commands;
  UrscriptCommandContainer cmdContainer;
  UrScriptPayload cmdPayload;

  auto graspApproachCommand = 
   std::make_unique<MoveJointCommand>(_cfg.graspApproachJoint);
  auto graspCommand = 
    std::make_unique<MoveJointCommand>(_cfg.graspJoint);
  cmdContainer.addCommand(std::move(graspApproachCommand))
              .addCommand(std::move(graspCommand));
  cmdPayload = UrScriptBuilder::construct(
    Motion::Bloom::RETURN_HOME_NAME, cmdContainer);
  commands.push_back( { cmdPayload } );

  auto placeApproachCommand = 
    std::make_unique<MoveJointCommand>(_cfg.placeApproachJoint);
  auto placeCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.placeCartesian);
  cmdContainer.addCommand(std::move(placeApproachCommand))
              .addCommand(std::move(placeCommand));
  cmdPayload = UrScriptBuilder::construct(
    Motion::Bloom::TRANSPORT_AND_PLACE_NAME, cmdContainer);
  commands.push_back( { cmdPayload } );

  auto placeRetractCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.placeApproachCartesian);
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
  cmdContainer.addCommand(std::move(placeRetractCommand))
              .addCommand(std::move(returnHomeCommand));
  cmdPayload = UrScriptBuilder::construct(
    Motion::Bloom::RETURN_HOME_NAME, cmdContainer);
  commands.push_back( { cmdPayload } );

  dispatchUscriptsAsyncCb(commands, cb);
}

void BloomMotionSequence::gracefulStop(const UscriptsBatchDoneCb& cb) {
  UrscriptCommandContainer cmdContainer;

  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
  cmdContainer.addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload = 
    UrScriptBuilder::construct(Motion::Bloom::RETURN_HOME_NAME, cmdContainer);

  const std::vector<UscriptCommand> commands {
    { cmdPayload }
  };

  dispatchUscriptsAsyncCb(commands, cb);
}

void BloomMotionSequence::recover(const UscriptsBatchDoneCb& cb) {
  std::vector<UscriptCommand> commands;
  UrscriptCommandContainer cmdContainer;

  //TODO fill on init
  constexpr bool holdingRose = false;
  if (holdingRose) {
    auto placeApproachCommand = 
      std::make_unique<MoveJointCommand>(_cfg.placeApproachJoint);
    auto placeCommand = 
      std::make_unique<MoveLinearCommand>(_cfg.placeCartesian);
    cmdContainer.addCommand(std::move(placeApproachCommand))
                .addCommand(std::move(placeCommand));
    const UrScriptPayload cmdPayload = UrScriptBuilder::construct(
      Motion::Bloom::TRANSPORT_AND_PLACE_NAME, cmdContainer);
    commands.push_back( { cmdPayload } );
  } else {
    auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
    cmdContainer.addCommand(std::move(returnHomeCommand));
    const UrScriptPayload cmdPayload = 
      UrScriptBuilder::construct(Motion::Bloom::RETURN_HOME_NAME, cmdContainer);
    commands.push_back( { cmdPayload } );
  }

  dispatchUscriptsAsyncCb(commands, cb);
}