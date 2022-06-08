#ifndef ROBO_CLEANER_COMMON_ROBOCLEANERTOPICS_H_
#define ROBO_CLEANER_COMMON_ROBOCLEANERTOPICS_H_

//System headers

//Other libraries headers

//Own components headers

//Forward declarations

constexpr auto ROBOT_MOVE_ACTION = "move_robot";

constexpr auto QUERY_INITIAL_ROBOT_STATE_SERVICE = "query_initial_robot_state";
constexpr auto QUERY_BATTERY_STATUS_SERVICE = "query_battery_status";
constexpr auto CHARGE_BATTERY_SERVICE = "charge_battery";

constexpr auto FIELD_MAP_REVEALED_TOPIC = "field_map_revealed";
constexpr auto FIELD_MAP_CLEANED_TOPIC = "field_map_cleaned";
constexpr auto SHUTDOWN_CONTROLLER_TOPIC = "shutdown_controller";

#endif /* ROBO_CLEANER_COMMON_ROBOCLEANERTOPICS_H_ */
