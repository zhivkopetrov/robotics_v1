//Corresponding header
#include "robo_collector_gui/RoboCollectorGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/config/RoboCollectorGuiConfig.h"

using namespace std::placeholders;

RoboCollectorGui::RoboCollectorGui(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorOutInterface(communicatorOutInterface) {

}

int32_t RoboCollectorGui::init(const std::any &cfg) {
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
    LOGERR("Error, parsing config failed");
    return FAILURE;
  }

  if (SUCCESS != initLayout(parsedCfg.layoutCfg)) {
    LOGERR("Error, initLayout() failed");
    return FAILURE;
  }

  const auto layoutInterface = _layout.produceInterface();
  if (SUCCESS !=
      initTurnHelper(layoutInterface, parsedCfg.layoutCfg.enemyFieldMarker)) {
    LOGERR("initTurnHelper() failed");
    return FAILURE;
  }

  if (SUCCESS != initControllerExternalBridge(layoutInterface)) {
    LOGERR("initControllerExternalBridge() failed");
    return FAILURE;
  }

  return SUCCESS;
}

void RoboCollectorGui::deinit() {
  _communicatorOutInterface.unregisterNodeCb(_controllerExternalBridge);
  _layout.deinit();
}

void RoboCollectorGui::draw() const {
  _layout.draw();
}

void RoboCollectorGui::handleEvent(const InputEvent &e) {
  _layout.handleEvent(e);
}

void RoboCollectorGui::process() {
  _collisionWatcher.process();
}

int32_t RoboCollectorGui::initLayout(const RoboCollectorLayoutConfig &cfg) {
  RoboCollectorLayoutOutInterface interface;
  interface.collisionWatcher = &_collisionWatcher;
  interface.isPlayerTurnActiveCb = std::bind(&TurnHelper::isPlayerTurnActive,
      &_turnHelper);
  interface.finishRobotActCb = std::bind(&TurnHelper::onRobotFinishAct,
      &_turnHelper, _1);

  if (SUCCESS != _layout.init(cfg, interface)) {
    LOGERR("Error in _layout.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorGui::initTurnHelper(
    const RoboCollectorLayoutInterface &interface, char fieldEnemyMarker) {
  TurnHelperConfig cfg;
  cfg.enablePlayerInputCb = interface.enablePlayerInputCb;
  cfg.getFieldDataCb = interface.getFieldDataCb;
  cfg.fieldEnemyMarker = fieldEnemyMarker;
  cfg.maxRobots = Defines::ROBOTS_CTN;
  cfg.robotActInterfaces = interface.robotActInterfaces;

  if (SUCCESS != _turnHelper.init(cfg)) {
    LOGERR("Error in _turnHelper.init()");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCollectorGui::initControllerExternalBridge(
    const RoboCollectorLayoutInterface &interface) {
  ControllerExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = _invokeActionEventCb;
  outInterface.moveButtonClickCb = interface.moveButtonClickCb;

  _controllerExternalBridge = std::make_shared<ControllerExternalBridge>();
  if (SUCCESS != _controllerExternalBridge->init(outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return FAILURE;
  }

  _communicatorOutInterface.registerNodeCb(_controllerExternalBridge);

  return SUCCESS;
}

