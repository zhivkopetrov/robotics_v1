#ifndef UR_CONTROL_GUI_URCONTROLGUI_H_
#define UR_CONTROL_GUI_URCONTROLGUI_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"
#include "game_engine/Game.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_gui/layout/UrControlGuiLayout.h"
#include "ur_control_gui/external_api/UrControlGuiExternalBridge.h"
#include "ur_control_gui/external_api/DashboardProvider.h"

//Forward declarations
class InputEvent;

class UrControlGui final : public Game {
public:
  friend class UrControlGuiInitHelper;

  UrControlGui(const Ros2CommunicatorInterface &communicatorOutInterface);

  ErrorCode init(const std::any &cfg) override;
  void deinit() override;

  void draw() const override;
  void handleEvent(const InputEvent &e) override;

  void process() override;

private:
  UrControlGuiLayout _layout;

  std::shared_ptr<DashboardProvider> _dashboardProvider;
  std::shared_ptr<UrControlGuiExternalBridge> _guiExternalBridge;
  Ros2CommunicatorInterface _communicatorInterface;
};

#endif /* UR_CONTROL_GUI_URCONTROLGUI_H_ */
