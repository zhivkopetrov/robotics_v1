#ifndef ROBO_MINER_GUI_ROBOMINERLAYOUT_H_
#define ROBO_MINER_GUI_ROBOMINERLAYOUT_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "robo_common/layout/RoboCommonLayout.h"

//Own components headers
#include "robo_miner_gui/layout/panels/PanelHandler.h"
#include "robo_miner_gui/layout/entities/CrystalHandler.h"

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

private:
  RoboCommonLayout _commonLayout;
  PanelHandler _panelHandler;
  CrystalHandler _crystalHandler;
};

#endif /* ROBO_MINER_GUI_ROBOMINERLAYOUT_H_ */
