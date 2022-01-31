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

using PlayerDamageCb = std::function<void(int32_t)>;
using IsPlayerTurnActiveCb = std::function<bool()>;
using SetFieldDataMarkerCb = std::function<void(const FieldPos&, char)>;
using ResetFieldDataMarkerCb = std::function<void(const FieldPos&)>;
using GetFieldDataCb = std::function<const FieldData&()>;
using RobotActCb = std::function<void(MoveType)>;
using GetRobotFieldPosCb = std::function<FieldPos()>;
using GetRobotDirCb = std::function<Direction()>;
using FinishRobotActCb = std::function<void(int32_t)>;
using EnablePlayerInputCb = std::function<void()>;
using MoveButtonClickCb = std::function<void(MoveType)>;
using IncrCollectedCoinsCb = std::function<void(int32_t)>;
using GameLostCb = std::function<void()>;
using GameWonCb = std::function<void()>;

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIFUNCTIONALDEFINES_H_ */
