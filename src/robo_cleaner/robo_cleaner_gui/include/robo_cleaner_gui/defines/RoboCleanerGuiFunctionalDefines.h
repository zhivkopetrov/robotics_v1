#ifndef ROBO_CLEANER_GUI_ROBOCLEANERGUIFUNCTIONALDEFINES_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERGUIFUNCTIONALDEFINES_H_

//System headers
#include <functional>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiDefines.h"

//Forward declarations
using EnergyDepletedCb = IndicatorDepletedCb;
using FieldMapRevelealedCb = std::function<void()>;
using FieldMapCleanedCb = std::function<void()>;

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERGUIFUNCTIONALDEFINES_H_ */
