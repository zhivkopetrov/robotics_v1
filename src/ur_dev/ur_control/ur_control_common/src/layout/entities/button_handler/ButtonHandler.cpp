//Corresponding header
#include "ur_control_common/layout/entities/button_handler/ButtonHandler.h"

//System headers

//Other libraries headers
#include "urscript_common/urscript/UrScriptParser.h"
#include "utils/input/InputEvent.h"
#include "utils/Log.h"

//Own components headers

ErrorCode ButtonHandler::initInternal(
  const ButtonHandlerConfig &cfg,
  const InvokeDashboardServiceCb& invokeDashboardServiceCb) {
  
  DashboardButtonConfig buttonCfg;
  CommandButtonConfig &baseCfg = buttonCfg.baseCfg;
  baseCfg.rsrcId = cfg.buttonRsrcId;
  baseCfg.fontRsrcId = cfg.buttonFontRsrcId;
  baseCfg.descriptionOffsetY = 25;

  const std::array<Point, DASHBOARD_BUTTONS_COUNT> buttonPositions { Point(100,
      880), Point(400, 880), Point(250, 700) };
  const std::array<std::string, DASHBOARD_BUTTONS_COUNT> buttonDescriptions {
      "Power on robot", "Power off robot", "Brake release" };
  constexpr std::array<DashboardCommand, DASHBOARD_BUTTONS_COUNT> buttonCommands {
      DashboardCommand::POWER_ON_ROBOT, DashboardCommand::POWER_OFF_ROBOT,
      DashboardCommand::BRAKE_RELEASE };

  for (int32_t i = 0; i < DASHBOARD_BUTTONS_COUNT; ++i) {
    baseCfg.pos = buttonPositions[i];
    baseCfg.descriptionText = buttonDescriptions[i];
    buttonCfg.command = buttonCommands[i];
    if (ErrorCode::SUCCESS != 
         _dashboardButtons[i].init(buttonCfg, invokeDashboardServiceCb)) {
      LOGERR("Error, _dashboardButtons[%d].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}