//Corresponding header
#include "ur_control_bloom/motion/BloomMotionSequence.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/motion/config/BloomMotionSequenceConfig.h"
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

BloomMotionSequence::BloomMotionSequence(
  const std::string& name, int32_t id, UrScriptHeaders&& headers) : 
    MotionSequence(name, id, std::move(headers)) {

}

ErrorCode BloomMotionSequence::init(const std::any& cfg) {
  auto err = ErrorCode::SUCCESS;
  [[maybe_unused]]const BloomMotionSequenceConfig parsedCfg = [&cfg, &err]() {
    BloomMotionSequenceConfig localCfg;
    try {
      localCfg = std::any_cast<const BloomMotionSequenceConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<BloomMotionSequenceConfig&> failed, %s", e.what());
      err = ErrorCode::FAILURE;
    }
    return localCfg;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing BloomMotionSequenceConfig failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != validateUrscriptHeaders()) {
    LOGERR("validateUrscriptHeaders() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void BloomMotionSequence::start(const MotionActionDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Bloom::GRASP_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Bloom::TRANSPORT_AND_PLACE_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void BloomMotionSequence::gracefulStop(const MotionActionDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING }
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void BloomMotionSequence::abort(const MotionActionDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Bloom::ABORT_NAME], 
      MotionExecutionPolicy::BLOCKING }
  };

  dispatchMotionsAsyncCb(commands, cb);
}

void BloomMotionSequence::recover(const MotionActionDoneCb& cb) {
  const std::vector<MotionCommand> commands {
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Bloom::TRANSPORT_AND_PLACE_NAME], 
      MotionExecutionPolicy::BLOCKING },
    { urScriptHeaders[Motion::Bloom::RETURN_HOME_NAME], 
      MotionExecutionPolicy::BLOCKING },
  };

  dispatchMotionsAsyncCb(commands, cb);
}

ErrorCode BloomMotionSequence::validateUrscriptHeaders() const {
  const std::vector<const char*> headers {
    Motion::Bloom::GRASP_NAME, 
    Motion::Bloom::TRANSPORT_AND_PLACE_NAME, 
    Motion::Bloom::RETURN_HOME_NAME,
    Motion::Bloom::ABORT_NAME
  };

  for (const char* header : headers) {
    if (urScriptHeaders.find(header) == urScriptHeaders.end()) {
      LOGERR(
        "Could not find UrScriptHeader [%s] in BloomMotionSequence", header);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}