//Corresponding header
#include "ur_control_common/motion/MotionSequence.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers

MotionSequence::MotionSequence(
  const std::string& name, int32_t id, 
  const DispatchMotionsAsyncCb& inputDispatchMotionsAsyncCb,
  UrScriptHeaders&& headers) : 
  urScriptHeaders(std::move(headers)), 
  dispatchMotionsAsyncCb(inputDispatchMotionsAsyncCb), _name(name), _id(id) {

}

int32_t MotionSequence::getId() const {
  return _id;
}

std::string MotionSequence::getName() const {
  return _name;
}