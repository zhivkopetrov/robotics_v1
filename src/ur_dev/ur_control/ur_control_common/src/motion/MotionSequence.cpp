//Corresponding header
#include "ur_control_common/motion/MotionSequence.h"

//System headers

//Other libraries headers

//Own components headers

MotionSequence::MotionSequence(const std::string& name, int32_t id) : 
  _name(name), _id(id) {

}

void MotionSequence::setDispatchUscriptsAsyncCb(
  const DispatchUscriptsAsyncCb& cb) {
  dispatchUscriptsAsyncCb = cb;
}

int32_t MotionSequence::getId() const {
  return _id;
}

std::string MotionSequence::getName() const {
  return _name;
}

void MotionSequence::abort(const UscriptsBatchDoneCb& batchDoneCb) {
  //NOTE: pin 0 is reserved for aborting (overriding URScripts)
  constexpr auto abortCmdPayload = 
    "def AbortMotion():\n\tset_standard_digital_out(0, False)\nend\n";
  const std::vector<UscriptCommand> commands {
    { abortCmdPayload }
  };

  dispatchUscriptsAsyncCb(commands, batchDoneCb);
}