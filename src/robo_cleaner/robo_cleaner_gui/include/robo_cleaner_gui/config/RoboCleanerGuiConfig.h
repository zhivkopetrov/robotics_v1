#ifndef ROBO_CLEANER_GUI_ROBOCLEANERGUICONFIG_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERGUICONFIG_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_cleaner_gui/layout/config/RoboCleanerLayoutConfig.h"
#include "robo_cleaner_gui/helpers/config/RoboCleanerSolutionValidatorConfig.h"
#include "robo_cleaner_gui/helpers/config/EnergyHandlerConfig.h"

//Forward declarations

struct RoboCleanerGuiConfig {
  RoboCleanerLayoutConfig layoutCfg;
  RoboCleanerSolutionValidatorConfig solutionValidatorConfig;
  EnergyHandlerConfig energyHandlerConfig;
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERGUICONFIG_H_ */
