#ifndef ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTCONFIG_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTCONFIG_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/config/RoboCommonLayoutConfig.h"

//Own components headers
#include "robo_cleaner_gui/layout/field/config/RoboCleanerFieldConfig.h"
#include "robo_cleaner_gui/layout/panels/config/PanelHandlerConfig.h"

//Forward declarations

struct RoboCleanerLayoutConfig {
  RoboCommonLayoutConfig commonLayoutCfg;

  RoboCleanerFieldConfig fieldCfg;
  PanelHandlerConfig panelHandlerCfg;
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERLAYOUTCONFIG_H_ */
