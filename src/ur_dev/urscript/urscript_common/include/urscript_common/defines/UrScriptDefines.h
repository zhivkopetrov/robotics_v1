#ifndef URSCRIPT_COMMON_URSCRIPTDEFINES_H_
#define URSCRIPT_COMMON_URSCRIPTDEFINES_H_

//System headers
#include <string>

//Other libraries headers

//Own components headers

//Forward declarations

using UrScriptPayload = std::string;

enum Ur10eRobotJoints {
  BASE_JOINT_IDX,
  SHOULDER_JOINT_IDX,
  ELBOW_JOINT_IDX,
  WRIST_1_JOINT_IDX,
  WRIST_2_JOINT_IDX,
  WRIST_3_JOINT_IDX,

  UR_10E_JOINTS_COUNT
};

enum class GripperType {
  HARDWARE,
  SIMULATION
};

enum class UrScriptCommandType {
  MOVE,
  GRIPPER,
  PARAM_CHANGE
};

struct UrScriptCommand {
  UrScriptCommand(UrScriptCommandType type) : _commandType(type) {}
  virtual ~UrScriptCommand() noexcept = default;
  virtual UrScriptPayload construct() const = 0;

  UrScriptCommandType getCommandType () const {
    return _commandType;
  }

private:
  UrScriptCommandType _commandType;
};

#endif /* URSCRIPT_COMMON_URSCRIPTDEFINES_H_ */