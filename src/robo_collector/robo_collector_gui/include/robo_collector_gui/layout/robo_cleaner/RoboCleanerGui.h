#ifndef ROBO_CLEANER_ROBOCLEANERGUI_H_
#define ROBO_CLEANER_ROBOCLEANERGUI_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "manager_utils/drawing/Text.h"

//Own components headers
#include "robo_collector_gui/layout/robo_cleaner/entities/Rubbish.h"
#include "robo_collector_gui/layout/robo_cleaner/panels/EnergyPanel.h"
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"

//Forward declarations
struct RoboCleanerGuiConfig;

class RoboCleanerGui {
public:
  int32_t init(const RoboCleanerGuiConfig &cfg);
  void deinit();

  void draw() const;

private:
  void createCounterText(const FieldPos &fieldPos, uint64_t fontId,
                         int32_t counterValue);

  void createObstacle(const FieldPos &fieldPos, uint64_t rsrcId);

  std::vector<Rubbish> _rubbish;
  std::vector<Text> _tileCounters;
  std::vector<Image> _obstacles;
  EnergyPanel _energyPanel;
  FieldData _fieldData;
};

#endif /* ROBO_CLEANER_ROBOCLEANERGUI_H_ */
