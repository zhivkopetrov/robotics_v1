#ifndef UR_CONTROL_COMMON_URCONTROLCOMMONDEFINES_H_
#define UR_CONTROL_COMMON_URCONTROLCOMMONDEFINES_H_

//System headers

//Other libraries headers
#include "urscript_common/defines/UrScriptDefines.h"

//Own components headers

//Forward declarations

constexpr auto STATUS_VISUALS_TEXTS_Y_OFFSET = 70;

enum class DashboardCommand {
  POWER_ON_ROBOT,
  POWER_OFF_ROBOT,
  BRAKE_RELEASE,
  GET_ROBOT_MODE,
  GET_SAFETY_MODE
};

//1:1 UR msg mapping
enum class RobotMode {
  Unknown = -2,
  NoController,
  Disconnected,
  ConfirmSafety,
  Booting,
  PowerOff,
  PowerOn,
  Idle,
  Backdrive,
  Running,
  UpdatingFirmware
};

//1:1 UR msg mapping
enum class SafetyMode {
  Unknown,
  Normal,
  Reduced,
  ProtectiveStop,
  Recovery,
  SafeguardStop,
  SystemEmergencyStop,
  RobotEmergencyStop,
  Violation,
  Fault,
  ValidateJointId,
  UndefinedSafetyMode,
  AutomaticModeSafeguardStop,
  SystemThreePositionEnablingStop
};

enum class MotionExecutionPolicy {
  //fire and forget from same thread
  NON_BLOCKING,

  //wait on result in a separate motion thread
  BLOCKING,
};

enum class MotionAction {
  START, 
  GRACEFUL_STOP, 
  ABORT, 
  RECOVER
};

enum GripperButtonDefines {
  ACTIVATE_GRIPPER_IDX, 
  OPEN_GRIPPER_IDX, 
  CLOSE_GRIPPER_IDX,
  GRIPPER_BUTTONS_COUNT
};

std::string toString(MotionAction action);

#endif /* UR_CONTROL_COMMON_URCONTROLCOMMONDEFINES_H_ */
