#ifndef ROBO_MINER_GUI_ROBOMINERGUIFUNCTIONALDEFINES_H_
#define ROBO_MINER_GUI_ROBOMINERGUIFUNCTIONALDEFINES_H_

//C system headers

//C++ system headers
#include <functional>

//Other libraries headers
#include "robo_common/layout/field/FieldPos.h"

//Own components headers
#include "robo_miner_gui/defines/RoboMinerGuiDefines.h"

//Forward declarations

using CrystalClickedCb = std::function<void(const FieldPos&)>;

#endif /* ROBO_MINER_GUI_ROBOMINERGUIFUNCTIONALDEFINES_H_ */
