#ifndef ROBO_MINER_ENERGYPANEL_H_
#define ROBO_MINER_ENERGYPANEL_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"

//Own components headers
#include "robo_collector_gui/robo_cleaner/panels/config/EnergyPanelConfig.h"

//Forward declarations

//TODO make HealthPanel and this class should be the same.
//     Rename to IndicatorPanel and reuse it
class EnergyPanel {
public:
  int32_t init(const EnergyPanelConfig &cfg);
  void draw() const;

private:
  Image _panel;
  Image _indicator;
};

#endif /* ROBO_MINER_ENERGYPANEL_H_ */
