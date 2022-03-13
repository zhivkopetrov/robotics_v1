#ifndef ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIFUNCTIONALDEFINES_H_
#define ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIFUNCTIONALDEFINES_H_

//System headers
#include <functional>

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiDefines.h"

//Forward declarations

using IsPlayerTurnActiveCb = std::function<bool()>;
using IncrCollectedCoinsCb = std::function<void(int32_t)>;

#endif /* ROBO_COLLECTOR_GUI_ROBOCOLLECTORGUIFUNCTIONALDEFINES_H_ */
