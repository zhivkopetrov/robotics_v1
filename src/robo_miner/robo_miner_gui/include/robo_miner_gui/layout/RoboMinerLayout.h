#ifndef ROBO_MINER_GUI_ROBOMINERLAYOUT_H_
#define ROBO_MINER_GUI_ROBOMINERLAYOUT_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>
#include <unordered_map>

//Other libraries headers
#include "robo_common/layout/RoboCommonLayout.h"

//Own components headers
#include "robo_miner_gui/layout/panels/PanelHandler.h"
#include "robo_miner_gui/layout/entities/Crystal.h"
#include "robo_miner_gui/layout/field/RoboMinerField.h"

//Forward declarations
class InputEvent;
class RoboMinerLayoutInitHelper;
struct RoboMinerLayoutConfig;
struct RoboMinerLayoutOutInterface;
struct RoboMinerLayoutInterface;

class RoboMinerLayout {
public:
  friend class RoboMinerLayoutInitHelper;

  int32_t init(const RoboMinerLayoutConfig& cfg,
               const RoboMinerLayoutOutInterface &outInterface,
               RoboMinerLayoutInterface& interface);
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent &e);

  void onCrystalClicked(const FieldPos& fieldPos);

private:
  RoboCommonLayout _commonLayout;
  std::vector<Crystal> _crystals;
  RoboMinerField _field;
  PanelHandler _panelHandler;

  //key = (currRow * maxCols) + currCol
  //value = relative crystal id
  std::unordered_map<int32_t, int32_t> _fieldPosToCrystalIdMapping;
};

#endif /* ROBO_MINER_GUI_ROBOMINERLAYOUT_H_ */
