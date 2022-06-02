//Corresponding header
#include "robo_common/layout/helpers/RoboCommonLayoutInitHelper.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/RoboCommonLayout.h"
#include "robo_common/layout/config/RoboCommonLayoutConfig.h"

ErrorCode RoboCommonLayoutInitHelper::init(
    const RoboCommonLayoutConfig &cfg,
    const RoboCommonLayoutOutInterface &outInterface,
    RoboCommonLayout &layout) {
  layout._map.create(cfg.mapRsrcId);

  if (ErrorCode::SUCCESS != initField(cfg.fieldCfg, outInterface, layout)) {
    LOGERR("initField failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initFogOfWar(cfg, outInterface, layout)) {
    LOGERR("initFogOfWar failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != layout._gameEndAnimator.init(
          outInterface.shutdownGameCb)) {
    LOGERR("_gameEndAnimator.init() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initPlayerRobot(cfg, outInterface, layout)) {
    LOGERR("initPlayerRobot failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCommonLayoutInitHelper::initField(
    const FieldConfig &cfg, const RoboCommonLayoutOutInterface &outInterface,
    RoboCommonLayout &layout) {
  const FieldOutInterface fieldOutIterface = {
      .objechApproachOverlayTriggeredCb =
          outInterface.objechApproachOverlayTriggeredCb, .collisionWatcher =
          outInterface.collisionWatcher };

  if (ErrorCode::SUCCESS != layout._field.init(cfg, fieldOutIterface)) {
    LOGERR("Error in _field.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCommonLayoutInitHelper::initFogOfWar(
    const RoboCommonLayoutConfig &layoutCfg,
    const RoboCommonLayoutOutInterface &outInterface,
    RoboCommonLayout &layout) {
  FogOfWarOutInterface fogOfWarOutInterface;
  fogOfWarOutInterface.collisionWatcher = outInterface.collisionWatcher;

  if (ErrorCode::SUCCESS != layout._fogOfWar.init(layoutCfg.fogOfWarConfig,
          fogOfWarOutInterface, layoutCfg.fieldCfg)) {
    LOGERR("Error in _fogOfWar.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode RoboCommonLayoutInitHelper::initPlayerRobot(
    const RoboCommonLayoutConfig &layoutCfg,
    const RoboCommonLayoutOutInterface &outInterface,
    RoboCommonLayout &layout) {
  using namespace std::placeholders;

  RobotOutInterface robotOutInterface;
  robotOutInterface.collisionWatcher = outInterface.collisionWatcher;
  robotOutInterface.playerDamageCb = outInterface.playerDamageCb;
  robotOutInterface.finishRobotActCb = outInterface.finishRobotActCb;
  robotOutInterface.playerRobotDamageCollisionCb =
      outInterface.playerRobotDamageCollisionCb;
  robotOutInterface.setFieldDataMarkerCb = std::bind(&Field::setFieldDataMarker,
      &layout._field, _1, _2);
  robotOutInterface.resetFieldDataMarkerCb = std::bind(
      &Field::resetFieldDataMarker, &layout._field, _1);
  robotOutInterface.getFieldDescriptionCb = std::bind(&Field::getDescription,
      &layout._field);

  const auto &baseCfg = layoutCfg.robotBaseCfg;
  RobotState initialState;
  RobotAnimatorConfigBase animatorCfgBase;
  animatorCfgBase.damageMarkerRsrcId = baseCfg.damageMarkerRsrcId;
  animatorCfgBase.robotRsrcId = baseCfg.playerRsrcId;
  animatorCfgBase.frameId = 0;
  animatorCfgBase.width = layoutCfg.fieldCfg.description.tileWidth;
  animatorCfgBase.height = layoutCfg.fieldCfg.description.tileHeight;
  initialState.robotId = RoboCommonDefines::PLAYER_ROBOT_IDX;
  animatorCfgBase.robotId = RoboCommonDefines::PLAYER_ROBOT_IDX;
  initialState.fieldPos = FieldPos(layoutCfg.fieldCfg.description.rows - 1,
      layoutCfg.fieldCfg.description.cols - 1);
  initialState.dir = Direction::UP;
  animatorCfgBase.moveAnimTimerId = baseCfg.moveAnimStartTimerId;
  animatorCfgBase.wallCollisionAnimTimerId =
      baseCfg.wallCollisionAnimStartTimerId;
  animatorCfgBase.robotCollisionAnimTimerId =
      baseCfg.robotCollisionAnimStartTimerId;
  animatorCfgBase.robotDamageAnimTimerId = baseCfg.robotDamageAnimStartTimerId;

  RobotConfig robotCfg;
  robotCfg.robotFieldMarkers = baseCfg.robotFieldMarkers;
  robotCfg.fieldMarker = layoutCfg.playerFieldMarker;

  if (ErrorCode::SUCCESS != layout._playerRobot.init(initialState, robotCfg,
          animatorCfgBase, robotOutInterface)) {
    LOGERR("Error in _playerRobot.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}
