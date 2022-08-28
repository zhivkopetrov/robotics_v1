#ifndef UR_CONTROL_GUI_URCONTROLGUITOPICS_H_
#define UR_CONTROL_GUI_URCONTROLGUITOPICS_H_

//System headers

//Other libraries headers

//Own components headers

//Forward declarations

constexpr auto URSCRIPT_TOPIC = "urscript";
constexpr auto ROBOT_MODE_TOPIC = "/io_and_status_controller/robot_mode";
constexpr auto SAFETY_MODE_TOPIC = "/io_and_status_controller/safety_mode";

constexpr auto DASHBOARD_CLIENT_POWER_ON_SERVICE = "dashboard_client/power_on";
constexpr auto DASHBOARD_CLIENT_POWER_OFF_SERVICE = "dashboard_client/power_off";
constexpr auto DASHBOARD_CLIENT_BRAKE_RELEASE_SERVICE = "dashboard_client/brake_release";
constexpr auto DASHBOARD_CLIENT_GET_ROBOT_MODE_SERVICE = "dashboard_client/get_robot_mode";
constexpr auto DASHBOARD_CLIENT_GET_SAFETY_MODE_SERVICE = "dashboard_client/get_safety_mode";

#endif /* UR_CONTROL_GUI_URCONTROLGUITOPICS_H_ */
