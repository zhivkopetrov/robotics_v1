//Corresponding header
#include "robo_common/layout/helpers/RoboCommonLayoutInitHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/RoboCommonLayout.h"
#include "robo_common/layout/config/RoboCommonLayoutConfig.h"

int32_t RoboCommonLayoutInitHelper::init(
    const RoboCommonLayoutConfig &cfg,
    const RoboCommonLayoutOutInterface &outInterface,
    RoboCommonLayout &layout) {
  layout._map.create(cfg.mapRsrcId);

  if (SUCCESS != layout._field.init(cfg.fieldCfg)) {
    LOGERR("Error in _field.init()");
    return FAILURE;
  }

  if (SUCCESS != layout._gameEndAnimator.init(outInterface.shutdownGameCb)) {
    LOGERR("_gameEndAnimator.init() failed");
    return FAILURE;
  }

  if (SUCCESS != initPlayerRobot(cfg, outInterface, layout)) {
    LOGERR("initPlayerRobot failed");
    return FAILURE;
  }

  return SUCCESS;
}

int32_t RoboCommonLayoutInitHelper::initPlayerRobot(
    const RoboCommonLayoutConfig &layoutCfg,
    const RoboCommonLayoutOutInterface &outInterface,
    RoboCommonLayout &layout) {
  using namespace std::placeholders;

  RobotOutInterface robotOutInterface;
  robotOutInterface.collisionWatcher = outInterface.collisionWatcher;
  robotOutInterface.playerDamageCb = outInterface.playerDamageCb;
  robotOutInterface.finishRobotActCb = outInterface.finishRobotActCb;
  robotOutInterface.setFieldDataMarkerCb =
      std::bind(&Field::setFieldDataMarker, &layout._field, _1, _2);
  robotOutInterface.resetFieldDataMarkerCb =
      std::bind(&Field::resetFieldDataMarker, &layout._field, _1);
  robotOutInterface.getFieldDescriptionCb =
      std::bind(&Field::getDescription, &layout._field);

  const auto &baseCfg = layoutCfg.robotBaseCfg;
  RobotConfig cfg;
  RobotAnimatorConfigBase animatorCfgBase;
  animatorCfgBase.damageMarkerRsrcId = baseCfg.damageMarkerRsrcId;
  animatorCfgBase.robotRsrcId = baseCfg.playerRsrcId;
  animatorCfgBase.frameId = 0;
  cfg.fieldMarker = layoutCfg.playerFieldMarker;
  cfg.enemyFieldMarker = layoutCfg.enemyFieldMarker;
  cfg.robotId = RoboCommonDefines::PLAYER_ROBOT_IDX;
  animatorCfgBase.robotId = RoboCommonDefines::PLAYER_ROBOT_IDX;
  cfg.fieldPos = FieldPos(layoutCfg.fieldCfg.description.rows - 1,
                          layoutCfg.fieldCfg.description.cols - 1);
  cfg.dir = Direction::UP;
  animatorCfgBase.moveAnimTimerId = baseCfg.moveAnimStartTimerId;
  animatorCfgBase.wallCollisionAnimTimerId =
      baseCfg.wallCollisionAnimStartTimerId;
  animatorCfgBase.robotCollisionAnimTimerId =
      baseCfg.robotCollisionAnimStartTimerId;
  animatorCfgBase.robotDamageAnimTimerId = baseCfg.robotDamageAnimStartTimerId;

  if (SUCCESS !=
      layout._playerRobot.init(cfg, animatorCfgBase, robotOutInterface)) {
    LOGERR("Error in _playerRobot.init()");
    return FAILURE;
  }

  return SUCCESS;
}
