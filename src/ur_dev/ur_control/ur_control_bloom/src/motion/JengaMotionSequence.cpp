//Corresponding header
#include "ur_control_bloom/motion/JengaMotionSequence.h"

//System headers

//Other libraries headers
#include "urscript_common/motion/UrScriptBuilder.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

JengaMotionSequence::JengaMotionSequence(
  const JengaMotionSequenceConfig& cfg,
  const std::string& name, int32_t id) : MotionSequence(name, id), _cfg(cfg) {
  populateUrscriptHeaders();
}

void JengaMotionSequence::start(const MotionCommandBatchDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Jenga::GRASP_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Jenga::TRANSPORT_AND_PLACE_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void JengaMotionSequence::gracefulStop(const MotionCommandBatchDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING }
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void JengaMotionSequence::recover(const MotionCommandBatchDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Jenga::TRANSPORT_AND_PLACE_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void JengaMotionSequence::populateUrscriptHeaders() {
  constexpr double accel = 1.0;
  constexpr double vel = 1.0;
  constexpr double blendingRadius = 0.0;

  MotionCommandContainer cmdContainer;

  auto graspApproachCommand = std::make_unique<MoveLinearCommand>(
    _cfg.graspApproachCartesian, accel, vel, blendingRadius);
  auto baseCenterACommand = std::make_unique<MoveLinearCommand>(
    _cfg.baseCenterACartesian, accel, vel, blendingRadius);
  cmdContainer.addCommand(std::move(graspApproachCommand))
              .addCommand(std::move(baseCenterACommand));
  urScriptHeaders[Motion::Jenga::GRASP_NAME] = UrScriptBuilder::construct(
    Motion::Jenga::GRASP_NAME, cmdContainer);

  auto transportApproachCommand = std::make_unique<MoveLinearCommand>(
    _cfg.graspApproachCartesian, accel, vel, blendingRadius);
  auto baseCenterBCommand = std::make_unique<MoveLinearCommand>(
    _cfg.baseCenterBCartesian, accel, vel, blendingRadius);

  cmdContainer.addCommand(std::move(transportApproachCommand))
              .addCommand(std::move(baseCenterBCommand));
  urScriptHeaders[Motion::Jenga::TRANSPORT_AND_PLACE_NAME] = 
    UrScriptBuilder::construct(
      Motion::Jenga::TRANSPORT_AND_PLACE_NAME, cmdContainer);

  auto returnHomeCommand = std::make_unique<MoveJointCommand>(
    _cfg.homeJoint, accel, vel, blendingRadius);
  cmdContainer.addCommand(std::move(returnHomeCommand));
  urScriptHeaders[Motion::Jenga::RETURN_HOME_NAME] = UrScriptBuilder::construct(
    Motion::Jenga::RETURN_HOME_NAME, cmdContainer);
}