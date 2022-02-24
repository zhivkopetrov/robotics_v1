#ifndef ROBO_CLEANER_GUI_ROBOCLEANERLAYOUT_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERLAYOUT_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "robo_common/layout/RoboCommonLayout.h"

//Own components headers
#include "robo_cleaner_gui/layout/panels/PanelHandler.h"
#include "robo_cleaner_gui/layout/entities/EntityHandler.h"

//Forward declarations
class RoboCleanerLayoutInitHelper;
struct RoboCleanerLayoutConfig;
struct RoboCleanerLayoutOutInterface;
struct RoboCleanerLayoutInterface;

class RoboCleanerLayout {
public:
  friend class RoboCleanerLayoutInitHelper;

  int32_t init(const RoboCleanerLayoutConfig& cfg,
               const RoboCleanerLayoutOutInterface &outInterface,
               RoboCleanerLayoutInterface& interface);
  void deinit();
  void draw() const;

  //TODO move to some dedicated class
  void onEnergyDepleted();

private:
  RoboCommonLayout _commonLayout;
  EntityHandler _entityHandler;
  PanelHandler _panelHandler;
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERLAYOUT_H_ */
