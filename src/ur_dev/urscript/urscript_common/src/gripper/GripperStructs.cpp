//Corresponding header
#include "urscript_common/gripper/GripperStructs.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers

namespace {
[[maybe_unused]]constexpr auto GRIPPER_SIMULATION_WARNING_MSG = 
  "# [Gripper commands are currently not supported in URScript]: ";

GripperType gGripperType = GripperType::SIMULATION;
}

void setGripperTypeGlobally(GripperType type) {
  gGripperType = type;
}

GripperCommandBase::GripperCommandBase()
  : UrScriptCommand(UrScriptCommandType::GRIPPER) {

}

GripperActivateCommand::GripperActivateCommand(GripperCommandPolicy policy) 
  : _policy(policy) {

}

UrScriptPayload GripperActivateCommand::construct() const {
  UrScriptPayload payload;
  if (GripperType::SIMULATION == gGripperType) {
    payload.append(GRIPPER_SIMULATION_WARNING_MSG);
  }

  payload.append("rq_activate");
  if (GripperCommandPolicy::BLOCKING == _policy) {
    payload.append("_and_wait");
  } 
  payload.append("()");

  return payload;
}

GripperActuateCommand::GripperActuateCommand(
  GripperActuateType type, GripperCommandPolicy policy) 
  : _actuateType(type), _policy(policy) {

}

UrScriptPayload GripperActuateCommand::construct() const {
  UrScriptPayload payload;
  if (GripperType::SIMULATION == gGripperType) {
    payload.append(GRIPPER_SIMULATION_WARNING_MSG);
  }

  payload.append(
    (GripperActuateType::OPEN == _actuateType ? 
      serializeOpenActuateType() : 
      serializeCloseActuateType())
  );

  return payload;
}

std::string GripperActuateCommand::serializeOpenActuateType() const {
  std::string data = "rq_open";
  if (GripperCommandPolicy::BLOCKING == _policy) {
    data.append("_and_wait");
  } 
  data.append("()");
  return data;
}

std::string GripperActuateCommand::serializeCloseActuateType() const {
  std::string data = "rq_close";
  if (GripperCommandPolicy::BLOCKING == _policy) {
    data.append("_and_wait");
  } 
  data.append("()");
  return data;
}

GripperParamCommand::GripperParamCommand(GripperParamType type, int32_t percent) 
  : _paramType(type), _percentValue(percent) {
    if ((0 > _percentValue) || 100 < _percentValue) {
      const std::string commandTypeStr = 
        GripperParamType::SPEED == _paramType ? "Speed" : "Force";
      LOGR("Received Invalid %sGripperCommand percent value: [%d]. "
           "Valid range is [0-100]. Defaulting to 100", commandTypeStr.c_str(),
           _percentValue);
        _percentValue = 100;
    }
}

UrScriptPayload GripperParamCommand::construct() const {
  UrScriptPayload payload;
  if (GripperType::SIMULATION == gGripperType) {
    payload.append(GRIPPER_SIMULATION_WARNING_MSG);
  }

  payload.append((
    GripperParamType::SPEED == _paramType ? 
      "rq_set_speed_norm(" : 
      "rq_set_force_norm("
    )
  ).append(std::to_string(_percentValue)).append(")");

  return payload;
}