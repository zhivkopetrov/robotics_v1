#ifndef UR_CONTROL_GUI_URCONTROLGUIDEFINES_H_
#define UR_CONTROL_GUI_URCONTROLGUIDEFINES_H_

enum class DashboardCommand {
  POWER_ON_ROBOT,
  POWER_OFF_ROBOT,
  BRAKE_RELEASE,
  GET_ROBOT_MODE,
  GET_ROBOT_SAFETY_MODE
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

#endif /* UR_CONTROL_GUI_URCONTROLGUIDEFINES_H_ */
