//Corresponding header
#include "ur_control_common/motion/MotionSequence.h"

//System headers

//Other libraries headers

//Own components headers

MotionSequence::MotionSequence(
  const std::string& name, int32_t id, 
  std::shared_ptr<UrScriptBuilder> urScriptBuilder) : 
  _name(name), _id(id), _urScriptBuilder(urScriptBuilder) {

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

void MotionSequence::abort(const UrscriptsBatchDoneCb& batchDoneCb) {
  //NOTE: pin 0 is reserved for aborting (overriding URScripts)
  constexpr auto abortCmdPayload = 
    "def AbortMotion():\n\tset_standard_digital_out(0, False)\nend\n";
  const std::vector<UrscriptCommand> commands {
    { abortCmdPayload }
  };

  dispatchUscriptsAsyncCb(commands, batchDoneCb);
}

UrScriptPayload MotionSequence::constructUrScript(
  std::string_view methodName, 
  UrScriptCommandContainer& commandContainer) const {
  return _urScriptBuilder->construct(methodName, commandContainer);
}