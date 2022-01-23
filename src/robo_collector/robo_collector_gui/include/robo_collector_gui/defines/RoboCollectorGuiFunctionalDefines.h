#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIFUNCTIONALDEFINES_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIFUNCTIONALDEFINES_H_

//C system headers

//C++ system headers
#include <vector>
#include <functional>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"
#include "robo_collector_gui/field/FieldPos.h"

//Forward declarations

using FieldData = std::vector<std::vector<char>>;

using CollisionCb = std::function<void(int32_t)>;
using SetFieldDataMarkerCb = std::function<void(const FieldPos&, char)>;
using ResetFieldDataMarkerCb = std::function<void(const FieldPos&)>;
using GetFieldDataCb = std::function<const FieldData&()>;
using RobotActCb = std::function<void(MoveType)>;
using IncrCollectedCoinsCb = std::function<void(int32_t)>;

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIFUNCTIONALDEFINES_H_ */
