//Corresponding header
#include "robo_collector_gui/helpers/RoboCollectorGuiInitHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/RoboCollectorGui.h"
#include "robo_collector_gui/config/RoboCollectorGuiConfig.h"

int32_t RoboCollectorGuiInitHelper::init(const std::any &cfg,
                                         RoboCollectorGui &gui) {
  int32_t err = SUCCESS;
  const auto parsedCfg = [&cfg, &err]() {
    RoboCollectorGuiConfig localCfg;
    try {
      localCfg = std::any_cast<const RoboCollectorGuiConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<GuiConfig&> failed, %s", e.what());
      err = FAILURE;
    }
    return localCfg;
  }();
  if (SUCCESS != err) {
    LOGERR("Error, parsing RoboCollectorGuiConfig failed");
    return FAILURE;
  }

  if (SUCCESS != initLayout(parsedCfg.layoutCfg, gui)) {
    LOGERR("Error, initLayout() failed");
    return FAILURE;
  }

  const auto layoutInterface = gui._layout.produceInterface();
  if (SUCCESS != initTurnHelper(layoutInterface,
          parsedCfg.layoutCfg.enemyFieldMarker, gui)) {
    LOGERR("initTurnHelper() failed");
    return FAILURE;
  }

  if (SUCCESS != initControllerExternalBridge(layoutInterface, gui)) {
    LOGERR("initControllerExternalBridge() failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorGuiInitHelper::initLayout(
    const RoboCollectorLayoutConfig &cfg, RoboCollectorGui &gui) {
  using namespace std::placeholders;

  RoboCollectorLayoutOutInterface interface;
  interface.collisionWatcher = &gui._collisionWatcher;
  interface.isPlayerTurnActiveCb = std::bind(&TurnHelper::isPlayerTurnActive,
      &gui._turnHelper);
  interface.finishRobotActCb = std::bind(&TurnHelper::onRobotFinishAct,
      &gui._turnHelper, _1);

  if (SUCCESS != gui._layout.init(cfg, interface)) {
    LOGERR("Error in _layout.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorGuiInitHelper::initTurnHelper(
    const RoboCollectorLayoutInterface &interface, char fieldEnemyMarker,
    RoboCollectorGui &gui) {
  TurnHelperConfig cfg;
  cfg.enablePlayerInputCb = interface.enablePlayerInputCb;
  cfg.getFieldDataCb = interface.getFieldDataCb;
  cfg.fieldEnemyMarker = fieldEnemyMarker;
  cfg.maxRobots = Defines::ROBOTS_CTN;
  cfg.robotActInterfaces = interface.robotActInterfaces;

  if (SUCCESS != gui._turnHelper.init(cfg)) {
    LOGERR("Error in _turnHelper.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorGuiInitHelper::initControllerExternalBridge(
    const RoboCollectorLayoutInterface &interface, RoboCollectorGui &gui) {
  ControllerExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = gui._invokeActionEventCb;
  outInterface.moveButtonClickCb = interface.moveButtonClickCb;

  gui._controllerExternalBridge = std::make_shared<ControllerExternalBridge>();
  if (SUCCESS != gui._controllerExternalBridge->init(outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return FAILURE;
  }

  gui._communicatorOutInterface.registerNodeCb(gui._controllerExternalBridge);

  return SUCCESS;
}

