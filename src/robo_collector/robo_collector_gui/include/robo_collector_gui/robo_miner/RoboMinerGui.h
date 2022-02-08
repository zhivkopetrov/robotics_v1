#ifndef ROBO_MINER_ROBOMINERGUI_H_
#define ROBO_MINER_ROBOMINERGUI_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>
#include <unordered_map>

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

  void onCrystalClicked(const FieldPos& fieldPos);

private:
  std::vector<Crystal> _crystals;
  RoboMinerField _field;

  //key = (currRow * maxCols) + currCol
  //value = relative crystal id
  std::unordered_map<int32_t, int32_t> _fieldPosToCrystalIdMapping;
};

#endif /* ROBO_MINER_ROBOMINERGUI_H_ */
