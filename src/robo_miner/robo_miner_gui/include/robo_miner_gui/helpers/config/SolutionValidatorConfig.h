#ifndef ROBO_MINER_GUI_SOLUTIONVALIDATORCONFIG_H_
#define ROBO_MINER_GUI_SOLUTIONVALIDATORCONFIG_H_

//System headers

//Other libraries headers

//Own components headers
#include "robo_miner_gui/defines/RoboMinerGuiDefines.h"

//Forward declarations

struct SolutionValidatorConfig {
  CrystalSequence longestSequence;
  int32_t targetMapTilesCount = 0;
};

#endif /* ROBO_MINER_GUI_SOLUTIONVALIDATORCONFIG_H_ */
