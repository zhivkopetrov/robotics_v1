//Corresponding header
#include "ur_control_bloom/motion/JengaMotionSequence.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/motion/config/JengaMotionSequenceConfig.h"
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

JengaMotionSequence::JengaMotionSequence(
  const std::string& name, int32_t id, UrScriptHeaders&& headers) : 
    MotionSequence(name, id, std::move(headers)) {

}

ErrorCode JengaMotionSequence::init(const std::any& cfg) {
  auto err = ErrorCode::SUCCESS;
  [[maybe_unused]]const JengaMotionSequenceConfig parsedCfg = [&cfg, &err]() {
    JengaMotionSequenceConfig localCfg;
    try {
      localCfg = std::any_cast<const JengaMotionSequenceConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<JengaMotionSequenceConfig&> failed, %s", e.what());
      err = ErrorCode::FAILURE;
    }
    return localCfg;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing JengaMotionSequenceConfig failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != validateUrscriptHeaders()) {
    LOGERR("validateUrscriptHeaders() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
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

ErrorCode JengaMotionSequence::validateUrscriptHeaders() const {
  const std::vector<const char*> headers {
    Motion::Jenga::GRASP_NAME, 
    Motion::Jenga::TRANSPORT_AND_PLACE_NAME, 
    Motion::Jenga::RETURN_HOME_NAME
  };

  for (const char* header : headers) {
    if (urScriptHeaders.find(header) == urScriptHeaders.end()) {
      LOGERR(
        "Could not find UrScriptHeader [%s] in JengaMotionSequence", header);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}