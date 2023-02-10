//Corresponding header
#include "ur_control_bloom/motion/BloomMotionSequence.h"

//System headers

//Other libraries headers
#include "urscript_common/motion/UrScriptBuilder.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

BloomMotionSequence::BloomMotionSequence(
  const BloomMotionSequenceConfig& cfg, const std::string& name, int32_t id) : 
    MotionSequence(name, id), _cfg(cfg) {
  populateUrscriptHeaders();
}

void BloomMotionSequence::start(const UscriptsBatchDoneCb& cb) {
  const std::vector<UscriptCommand> commands {
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME] },
    { urScriptHeaders[Motion::Bloom::GRASP_NAME] },
    { urScriptHeaders[Motion::Bloom::TRANSPORT_AND_PLACE_NAME] },
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME] }
  };

  dispatchUscriptsAsyncCb(commands, cb);
}

void BloomMotionSequence::gracefulStop(const UscriptsBatchDoneCb& cb) {
  const std::vector<UscriptCommand> commands {
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME] }
  };

  dispatchUscriptsAsyncCb(commands, cb);
}

void BloomMotionSequence::recover(const UscriptsBatchDoneCb& cb) {
  const std::vector<UscriptCommand> commands {
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME] } ,
    { urScriptHeaders[Motion::Bloom::TRANSPORT_AND_PLACE_NAME] },
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME] }
  };

  dispatchUscriptsAsyncCb(commands, cb);
}

void BloomMotionSequence::populateUrscriptHeaders() {
  constexpr double accel = 1.0;
  constexpr double vel = 1.0;
  constexpr double blendingRadius = 0.0;

  MotionCommandContainer cmdContainer;

  auto graspApproachCommand = std::make_unique<MoveJointCommand>(
    _cfg.graspApproachJoint, accel, vel, blendingRadius);
  auto graspCommand = std::make_unique<MoveJointCommand>(
    _cfg.graspJoint, accel, vel, blendingRadius);
  cmdContainer.addCommand(std::move(graspApproachCommand))
              .addCommand(std::move(graspCommand));
  urScriptHeaders[Motion::Bloom::GRASP_NAME] = UrScriptBuilder::construct(
    Motion::Bloom::GRASP_NAME, cmdContainer);

  auto placeApproachCommand = std::make_unique<MoveJointCommand>(
    _cfg.placeApproachJoint, accel, vel, blendingRadius);
  auto placeCommand = std::make_unique<MoveLinearCommand>(
    _cfg.placeCartesian, accel, vel, blendingRadius);
  cmdContainer.addCommand(std::move(placeApproachCommand))
              .addCommand(std::move(placeCommand));
  urScriptHeaders[Motion::Bloom::TRANSPORT_AND_PLACE_NAME] = 
    UrScriptBuilder::construct(
      Motion::Bloom::TRANSPORT_AND_PLACE_NAME, cmdContainer);

  auto placeRetractCommand = std::make_unique<MoveLinearCommand>(
    _cfg.placeApproachCartesian, accel, vel, blendingRadius);
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(
    _cfg.homeJoint, accel, vel, blendingRadius);
  cmdContainer.addCommand(std::move(placeRetractCommand))
              .addCommand(std::move(returnHomeCommand));
  urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME] = UrScriptBuilder::construct(
    Motion::Bloom::RETURN_HOME_NAME, cmdContainer);
}