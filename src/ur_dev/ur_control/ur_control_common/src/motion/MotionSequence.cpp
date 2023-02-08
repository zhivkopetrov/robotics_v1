//Corresponding header
#include "ur_control_common/motion/MotionSequence.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers

MotionSequence::MotionSequence(
  const std::string& name, int32_t id, UrScriptHeaders&& headers) : 
  urScriptHeaders(std::move(headers)), _name(name), _id(id) {

}

void MotionSequence::setDispatchMotionsAsyncCb(
  const DispatchMotionsAsyncCb& cb) {
  dispatchMotionsAsyncCb = cb;
}

int32_t MotionSequence::getId() const {
  return _id;
}

std::string MotionSequence::getName() const {
  return _name;
}

void MotionSequence::abort(const MotionCommandBatchDoneCb& cb) {
  //NOTE: pin 0 is reserved for aborting (overriding URScripts)
  constexpr auto abortCmdPayload = 
    "def AbortMotion():\n\tset_standard_digital_out(0, False)\nend\n";
  const std::vector<MotionCommand> commands {
    { abortCmdPayload, MotionExecutionPolicy::BLOCKING }
  };

  dispatchMotionsAsyncCb(commands, cb);
}