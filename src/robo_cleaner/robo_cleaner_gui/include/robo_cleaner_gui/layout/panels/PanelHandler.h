#ifndef ROBO_CLEANER_GUI_PANELS_PANELHANDLER_H_
#define ROBO_CLEANER_GUI_PANELS_PANELHANDLER_H_

//System headers

//Other libraries headers
#include "robo_common/layout/panels/TimePanel.h"
#include "robo_common/layout/panels/IndicatorPanel.h"
#include "robo_common/layout/panels/NumberCounterPanel.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiFunctionalDefines.h"
#include "robo_cleaner_gui/layout/panels/config/PanelHandlerConfig.h"

//Forward declarations

struct PanelHandlerOutInterface {
  StartGameLostAnimCb startGameLostAnimCb;
  StartAchievementWonAnimCb startAchievementWonAnimCb;
  ShutdownControllerCb shutdownControllerCb;
  EnergyDepletedCb energyDepletedCb;
  FieldMapRevelealedCb fieldMapRevelealedCb;
  FieldMapCleanedCb fieldMapCleanedCb;
};

class PanelHandler {
public:
  ErrorCode init(const PanelHandlerConfig &cfg,
                 const PanelHandlerOutInterface &interface);
  void draw() const;
  void modifyHealthIndicator(int32_t delta);
  void modifyEnergyIndicator(int32_t delta);

  void onTileRevealed();
  void onTileCleaned();

  int32_t getHealthIndicatorValue() const;

private:
  ErrorCode validateInterface(const PanelHandlerOutInterface &interface) const;

  NumberCounterPanel _tilePanel;
  NumberCounterPanel _rubbishPanel;
  IndicatorPanel _healthPanel;
  IndicatorPanel _energyPanel;
};

#endif /* ROBO_CLEANER_GUI_PANELS_PANELHANDLER_H_ */
