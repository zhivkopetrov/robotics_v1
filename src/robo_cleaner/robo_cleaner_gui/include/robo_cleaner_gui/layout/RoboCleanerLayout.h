#ifndef ROBO_CLEANER_GUI_ROBOCLEANERLAYOUT_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERLAYOUT_H_

//System headers
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

  ErrorCode init(const RoboCleanerLayoutConfig& cfg,
                 const RoboCleanerLayoutOutInterface &outInterface,
                 RoboCleanerLayoutInterface& interface);
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent &e);

  void process();

private:
  void produceInterface(RoboCleanerLayoutInterface& interface);

  RoboCommonLayout _commonLayout;
  EntityHandler _entityHandler;
  PanelHandler _panelHandler;
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERLAYOUT_H_ */
