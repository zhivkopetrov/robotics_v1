#ifndef ROBO_CLEANER_GUI_ROBOCLEANERGUIFUNCTIONALDEFINES_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERGUIFUNCTIONALDEFINES_H_

//System headers
#include <functional>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Own components headers
#include <rclcpp_action/rclcpp_action.hpp>
#include "robo_cleaner_interfaces/action/robot_move.hpp"
#include "robo_cleaner_gui/defines/RoboCleanerGuiDefines.h"

//Forward declarations

using RobotMove = robo_cleaner_interfaces::action::RobotMove;
using GoalHandleRobotMove = rclcpp_action::ServerGoalHandle<RobotMove>;

using EnergyDepletedCb = IndicatorDepletedCb;
using FieldMapRevelealedCb = std::function<void()>;
using FieldMapCleanedCb = std::function<void()>;
using ReportMoveProgressCb = std::function<void(const MoveProgress&)>;
using ReportRobotStartingActCb = std::function<void(MoveType)>;
using AcceptGoalCb = std::function<void(const std::shared_ptr<GoalHandleRobotMove> &)>;

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERGUIFUNCTIONALDEFINES_H_ */
