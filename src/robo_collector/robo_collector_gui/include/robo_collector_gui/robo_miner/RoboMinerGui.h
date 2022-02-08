#ifndef ROBO_MINER_ROBOMINERGUI_H_
#define ROBO_MINER_ROBOMINERGUI_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/robo_miner/entities/Crystal.h"
#include "robo_collector_gui/robo_miner/field/RoboMinerField.h"

//Forward declarations
class InputEvent;
struct RoboMinerGuiConfig;

class RoboMinerGui {
public:
  int32_t init(const RoboMinerGuiConfig& cfg);
  void deinit();

  void draw() const;
  void handleEvent(const InputEvent &e);

private:
  std::vector<Crystal> _crystals;
  RoboMinerField _field;
};

#endif /* ROBO_MINER_ROBOMINERGUI_H_ */
