#ifndef URSCRIPT_COMMON_GRIPPERTRUCTS_H_
#define URSCRIPT_COMMON_GRIPPERTRUCTS_H_

//System headers

//Other libraries headers

//Own components headers
#include "urscript_common/defines/UrScriptDefines.h"

//Forward declarations

enum GripperCommandPolicy {
  //command will be waited to complete inside URScript before moving on to next one
  NON_BLOCKING,

  // execute command in an async manner in URScript and move to the next one
  BLOCKING 
};

enum GripperActuateType {
  OPEN,
  CLOSE
};

enum GripperParamType {
  SPEED,
  FORCE
};

//used to globally the gripper type
//this method is a temporary measure to easily enable/disable hardware support
//for the gripper
//currently Robotiq URScripts are only available on a real hardware
//The URSim docker needs to be extended and Robotiq URCaps to be installed
void setGripperTypeGlobally(GripperType type); 

struct GripperCommandBase : public UrScriptCommand {
  GripperCommandBase();
};

struct GripperActivateCommand final : public GripperCommandBase {
  GripperActivateCommand(
    GripperCommandPolicy policy = GripperCommandPolicy::BLOCKING);

  UrScriptPayload construct() const override;

private:
  const GripperCommandPolicy _policy;
};

struct GripperActuateCommand final : public GripperCommandBase {
  GripperActuateCommand(
    GripperActuateType type, 
    GripperCommandPolicy policy = GripperCommandPolicy::BLOCKING);

  UrScriptPayload construct() const override;

private:
  std::string serializeOpenActuateType() const;
  std::string serializeCloseActuateType() const;

  const GripperActuateType _actuateType;
  const GripperCommandPolicy _policy;
};

// percent accepts values in range [0-100]
struct GripperParamCommand final : public GripperCommandBase {
  GripperParamCommand(GripperParamType type, int32_t percent = 100);

  UrScriptPayload construct() const override;

private:
  const GripperParamType _paramType;
  int32_t _percentValue;
};

#endif /* URSCRIPT_COMMON_GRIPPERTRUCTS_H_ */