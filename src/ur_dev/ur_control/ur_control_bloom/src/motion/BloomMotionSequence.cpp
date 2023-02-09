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

void BloomMotionSequence::start(const MotionCommandBatchDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Bloom::GRASP_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Bloom::TRANSPORT_AND_PLACE_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING }
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void BloomMotionSequence::gracefulStop(const MotionCommandBatchDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING }
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void BloomMotionSequence::recover(const MotionCommandBatchDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING } ,
    { urScriptHeaders[Motion::Bloom::TRANSPORT_AND_PLACE_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING }
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void BloomMotionSequence::populateUrscriptHeaders() {
  //TODO parse from files
  const AngleAxis orientation(0.0, -3.16, 0.0);
  constexpr double accel = 1.0;
  constexpr double vel = 1.0;
  constexpr double blendingRadius = 0.0;

  const WaypointCartesian graspPose(Point3d(-0.5, -0.4, 0.2), orientation);
  auto graspCommand = std::make_unique<MoveLinearCommand>(
    graspPose, accel, vel, blendingRadius);

  const WaypointCartesian transportPose(Point3d(0.0, -0.4, 0.2), orientation);
  auto transportCommand = std::make_unique<MoveLinearCommand>(
    transportPose, accel, vel, blendingRadius);

  const WaypointCartesian homePose(Point3d(0.5, -0.4, 0.6), orientation);
  auto returnHomeCommand = std::make_unique<MoveLinearCommand>(
    homePose, accel, vel, blendingRadius);

  MotionCommandContainer cmdContainer;

  cmdContainer.addCommand(std::move(graspCommand));
  urScriptHeaders[Motion::Bloom::GRASP_NAME] = UrScriptBuilder::construct(
    Motion::Bloom::GRASP_NAME, cmdContainer);

  cmdContainer.addCommand(std::move(transportCommand));
  urScriptHeaders[Motion::Bloom::TRANSPORT_AND_PLACE_NAME] = 
    UrScriptBuilder::construct(
      Motion::Bloom::TRANSPORT_AND_PLACE_NAME, cmdContainer);

  cmdContainer.addCommand(std::move(returnHomeCommand));
  urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME] = UrScriptBuilder::construct(
    Motion::Bloom::RETURN_HOME_NAME, cmdContainer);
}