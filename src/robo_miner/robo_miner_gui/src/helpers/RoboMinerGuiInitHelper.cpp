//Corresponding header
#include "robo_miner_gui/helpers/RoboMinerGuiInitHelper.h"

//System headers

//Other libraries headers
#include "utils/log/Log.h"

//Own components headers
#include "robo_miner_gui/layout/helpers/RoboMinerLayoutInterfaces.h"
#include "robo_miner_gui/RoboMinerGui.h"
#include "robo_miner_gui/config/RoboMinerGuiConfig.h"

ErrorCode RoboMinerGuiInitHelper::init(const std::any &cfg, RoboMinerGui &gui) {
  auto err = ErrorCode::SUCCESS;
  const auto parsedCfg = [&cfg, &err]() {
    RoboMinerGuiConfig localCfg;
    try {
      localCfg = std::any_cast<const RoboMinerGuiConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<RoboMinerGuiConfig&> failed, %s", e.what());
      err = ErrorCode::FAILURE;
    }
    return localCfg;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing RoboCollectorGuiConfig failed");
    return ErrorCode::FAILURE;
  }

  //allocate memory for the external bridge in order to attach it's callbacks
  gui._controllerExternalBridge =
      std::make_shared<MinerControllerExternalBridge>();

  RoboMinerLayoutInterface layoutInterface;
  if (ErrorCode::SUCCESS !=
      initLayout(parsedCfg.layoutCfg, layoutInterface, gui)) {
    LOGERR("Error, initLayout() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != gui._movementWatcher.init(
          layoutInterface.commonLayoutInterface.getPlayerSurroundingTilesCb)) {
    LOGERR("_movementWatcher.init() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS !=
      initSolutionValidator(parsedCfg.solutionValidatorCfg, layoutInterface,
          gui)) {
    LOGERR("initSolutionValidator() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS !=
      initControllerExternalBridge(layoutInterface, gui)) {
    LOGERR("initControllerExternalBridge() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboMinerGuiInitHelper::initLayout(const RoboMinerLayoutConfig &cfg,
                                           RoboMinerLayoutInterface &interface,
                                           RoboMinerGui &gui) {
  using namespace std::placeholders;

  RoboMinerLayoutOutInterface outInterface;
  outInterface.collisionWatcher = &gui._collisionWatcher;
  outInterface.finishRobotActCb = std::bind(&MovementWatcher::changeState,
      &gui._movementWatcher, _1, _2);
  outInterface.shutdownGameCb = gui._systemShutdownCb;
  outInterface.takeScreenshotCb = gui._takeScreenshotCb;
  outInterface.shutdownControllerCb = std::bind(
      &MinerControllerExternalBridge::publishShutdownController,
      gui._controllerExternalBridge.get());
  outInterface.fieldMapRevelealedCb = std::bind(
      &MinerControllerExternalBridge::publishFieldMapRevealed,
      gui._controllerExternalBridge.get());

  if (ErrorCode::SUCCESS != gui._layout.init(cfg, outInterface, interface)) {
    LOGERR("Error in _layout.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboMinerGuiInitHelper::initSolutionValidator(
    const SolutionValidatorConfig &cfg,
    const RoboMinerLayoutInterface &interface, RoboMinerGui &gui) {
  SolutionValidatorOutInterface outInterface;
  outInterface.getFieldDescriptionCb =
      interface.commonLayoutInterface.getFieldDescriptionCb;
  outInterface.getRobotStateCb =
      interface.commonLayoutInterface.playerRobotActInterface.getRobotStateCb;
  outInterface.getPlayerSurroundingTilesCb =
      interface.commonLayoutInterface.getPlayerSurroundingTilesCb;

  if (ErrorCode::SUCCESS != gui._solutionValidator.init(cfg, outInterface)) {
    LOGERR("_solutionValidator.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboMinerGuiInitHelper::initControllerExternalBridge(
    const RoboMinerLayoutInterface &interface, RoboMinerGui &gui) {
  MinerControllerExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = gui._invokeActionEventCb;
  outInterface.robotActCb =
      interface.commonLayoutInterface.playerRobotActInterface.actCb;
  outInterface.toggleHelpPageCb =
      interface.commonLayoutInterface.toggleHelpPageCb;
  outInterface.toggleDebugInfoCb =
      interface.commonLayoutInterface.toggleDebugInfoCb;
  outInterface.setDebugMsgCb = interface.commonLayoutInterface.setDebugMsgCb;
  outInterface.setUserDataCb = interface.commonLayoutInterface.setUserDataCb;
  outInterface.startAchievementWonAnimCb =
      interface.commonLayoutInterface.startAchievementWonAnimCb;
  outInterface.startGameLostAnimCb =
      interface.commonLayoutInterface.startGameLostAnimCb;
  outInterface.tileReleavedCb = interface.tileReleavedCb;
  outInterface.revealFogOfWarTilesCb =
      interface.commonLayoutInterface.revealFogOfWarTilesCb;
  outInterface.crystalMinedCb = interface.crystalMinedCb;
  outInterface.movementWatcher = &gui._movementWatcher;
  outInterface.solutionValidator = &gui._solutionValidator;

  if (ErrorCode::SUCCESS != gui._controllerExternalBridge->init(outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

