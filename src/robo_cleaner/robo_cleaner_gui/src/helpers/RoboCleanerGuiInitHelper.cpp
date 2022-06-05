//Corresponding header
#include "robo_cleaner_gui/helpers/RoboCleanerGuiInitHelper.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/layout/helpers/RoboCleanerLayoutInterfaces.h"
#include "robo_cleaner_gui/RoboCleanerGui.h"
#include "robo_cleaner_gui/config/RoboCleanerGuiConfig.h"

using namespace std::placeholders;

ErrorCode RoboCleanerGuiInitHelper::init(const std::any &cfg,
                                         RoboCleanerGui &gui) {
  auto err = ErrorCode::SUCCESS;
  const auto parsedCfg = [&cfg, &err]() {
    RoboCleanerGuiConfig localCfg;
    try {
      localCfg = std::any_cast<const RoboCleanerGuiConfig&>(cfg);
    } catch (const std::bad_any_cast &e) {
      LOGERR("std::any_cast<RoboCleanerGuiConfig&> failed, %s", e.what());
      err = ErrorCode::FAILURE;
    }
    return localCfg;
  }();
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, parsing RoboCollectorGuiConfig failed");
    return ErrorCode::FAILURE;
  }

  //allocate memory for the external bridge in order to attach it's callbacks
  gui._controllerExternalBridge = std::make_shared<
      CleanerControllerExternalBridge>();

  RoboCleanerLayoutInterface layoutInterface;
  if (ErrorCode::SUCCESS != initLayout(parsedCfg.layoutCfg, layoutInterface,
          gui)) {
    LOGERR("Error, initLayout() failed");
    return ErrorCode::FAILURE;
  }

  const ResetControllerStatusCb resetControllerStatusCb = std::bind(
      &CleanerControllerExternalBridge::resetControllerStatus,
      gui._controllerExternalBridge.get());
  if (ErrorCode::SUCCESS != gui._movementReporter.init(
          resetControllerStatusCb)) {
    LOGERR("Error, _movementReporter.init() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initMovementWatcher(parsedCfg.layoutCfg,
          layoutInterface, gui)) {
    LOGERR("initMovementWatcher() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initSolutionValidator(
          parsedCfg.solutionValidatorConfig, layoutInterface, gui)) {
    LOGERR("initSolutionValidator() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initControllerExternalBridge(layoutInterface,
          gui)) {
    LOGERR("initControllerExternalBridge() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCleanerGuiInitHelper::initLayout(
    const RoboCleanerLayoutConfig &cfg, RoboCleanerLayoutInterface &interface,
    RoboCleanerGui &gui) {
  RoboCleanerLayoutOutInterface outInterface;
  outInterface.collisionWatcher = &gui._collisionWatcher;
  outInterface.finishRobotActCb = std::bind(&MovementWatcher::changeState,
      &gui._movementWatcher, _1, _2);
  outInterface.playerRobotDamageCollisionCb = std::bind(
      &MovementWatcher::cancelFeedbackReporting, &gui._movementWatcher);
  outInterface.fieldMapRevelealedCb = std::bind(
      &CleanerControllerExternalBridge::publishFieldMapRevealed,
      gui._controllerExternalBridge.get());
  outInterface.fieldMapCleanedCb = std::bind(
      &CleanerControllerExternalBridge::publishFieldMapCleaned,
      gui._controllerExternalBridge.get());
  outInterface.shutdownGameCb = std::bind(
      &CleanerControllerExternalBridge::publishShutdownController,
      gui._controllerExternalBridge.get());
  outInterface.objectApproachOverlayTriggeredCb = std::bind(
      &MovementWatcher::onObstacleApproachTrigger, &gui._movementWatcher, _1);

  if (ErrorCode::SUCCESS != gui._layout.init(cfg, outInterface, interface)) {
    LOGERR("Error in _layout.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCleanerGuiInitHelper::initMovementWatcher(
    const RoboCleanerLayoutConfig &cfg,
    const RoboCleanerLayoutInterface &interface, RoboCleanerGui &gui) {
  MovementWatcherOutInterface outInterface;
  const auto &commonLayoutInterface = interface.commonLayoutInterface;
  outInterface.getRobotAbsolutePosCb =
      commonLayoutInterface.playerRobotActInterface.getRobotAbsolutePosCb;
  outInterface.getRobotRotationAngleCb =
      commonLayoutInterface.playerRobotActInterface.getRobotRotationAngleCb;
  outInterface.getRobotStateCb =
      commonLayoutInterface.playerRobotActInterface.getRobotStateCb;
  outInterface.reportMoveProgressCb = std::bind(
      &MovementReporter::reportProgress, &gui._movementReporter, _1);
  outInterface.getFieldDescriptionCb =
      commonLayoutInterface.getFieldDescriptionCb;
  outInterface.setFieldDataMarkerCb =
      commonLayoutInterface.setFieldDataMarkerCb;
  outInterface.modifyRubbishWidgetCb = interface.modifyRubbishWidgetCb;
  outInterface.solutionValidator = &gui._solutionValidator;

  const auto &fieldDescr = cfg.commonLayoutCfg.fieldCfg.description;
  MovementWatcherConfig movementWatcherConfigCfg;
  movementWatcherConfigCfg.tileWidth = fieldDescr.tileWidth;
  movementWatcherConfigCfg.tileHeight = fieldDescr.tileHeight;
  if (ErrorCode::SUCCESS != gui._movementWatcher.init(movementWatcherConfigCfg,
          outInterface)) {
    LOGERR("Error in _movementWatcher.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCleanerGuiInitHelper::initSolutionValidator(
    const RoboCleanerSolutionValidatorConfig &cfg,
    const RoboCleanerLayoutInterface &interface, RoboCleanerGui &gui) {
  RoboCleanerSolutionValidatorOutInterface outInterface;
  outInterface.getFieldDescriptionCb =
      interface.commonLayoutInterface.getFieldDescriptionCb;
  outInterface.getRobotStateCb =
      interface.commonLayoutInterface.playerRobotActInterface.getRobotStateCb;

  if (ErrorCode::SUCCESS != gui._solutionValidator.init(cfg, outInterface)) {
    LOGERR("_solutionValidator.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCleanerGuiInitHelper::initControllerExternalBridge(
    const RoboCleanerLayoutInterface &interface, RoboCleanerGui &gui) {
  CleanerControllerExternalBridgeOutInterface outInterface;
  outInterface.invokeActionEventCb = gui._invokeActionEventCb;
  outInterface.robotActInterface =
      interface.commonLayoutInterface.playerRobotActInterface;
  outInterface.systemShutdownCb = gui._systemShutdownCb;
  outInterface.acceptGoalCb = std::bind(&MovementReporter::acceptGoal,
      &gui._movementReporter, _1);
  outInterface.reportRobotStartingActCb = std::bind(
      &MovementWatcher::onRobotStartingAct, &gui._movementWatcher, _1, _2);
  outInterface.cancelFeedbackReportingCb = std::bind(
      &MovementWatcher::cancelFeedbackReporting, &gui._movementWatcher);
  outInterface.solutionValidator = &gui._solutionValidator;

  if (ErrorCode::SUCCESS != gui._controllerExternalBridge->init(outInterface)) {
    LOGERR("Error in _controllerExternalBridge.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

