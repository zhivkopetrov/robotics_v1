#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIFUNCTIONALDEFINES_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIFUNCTIONALDEFINES_H_

//C system headers

//C++ system headers
#include <functional>

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"

//Forward declarations

using IsPlayerTurnActiveCb = std::function<bool()>;
using EnablePlayerInputCb = std::function<void()>;
using MoveButtonClickCb = std::function<void(MoveType)>;
using IncrCollectedCoinsCb = std::function<void(int32_t)>;
using SettingActivatedCb = std::function<void(GameType)>;
using HelpActivatedCb = std::function<void()>;

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIFUNCTIONALDEFINES_H_ */
