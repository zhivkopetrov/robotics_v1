//Corresponding header
#include "robo_common/layout/field/obstacles/ObstacleHandler.h"

//System headers
#include <array>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/field/FieldUtils.h"
#include "robo_common/layout/field/config/ObstacleHandlerConfig.h"

ErrorCode ObstacleHandler::init(
    const ObstacleHandlerConfig &cfg, const FieldDescription &fieldDescr,
    const std::vector<FieldPos> &innerObstaclePositions,
    const ObstacleHandlerOutInterface &interface) {
  const ObstacleOutInterface obstacleOutInterface = {
    .objectApproachOverlayTriggeredCb =
        interface.objechApproachOverlayTriggeredCb,
    .containerRedrawCb = interface.containerRedrawCb,
    .collisionWatcher = interface.collisionWatcher
  };

  if (ErrorCode::SUCCESS != initInnerObstacles(cfg, fieldDescr,
          innerObstaclePositions, obstacleOutInterface)) {
    LOGERR("Error, initInnerObstacles() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initOuterObstacles(cfg, fieldDescr,
      obstacleOutInterface)) {
    LOGERR("Error, initOuterObstacles() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void ObstacleHandler::draw() const {
  for (const auto &obstacle : _innerObstacles) {
    obstacle.draw();
  }

  for (const auto &obstacle : _outerObstacles) {
    obstacle.draw();
  }
}

void ObstacleHandler::drawOnFbo(Fbo &fbo) const {
  for (const auto &obstacle : _innerObstacles) {
    obstacle.drawOnFbo(fbo);
  }

  for (const auto &obstacle : _outerObstacles) {
    obstacle.drawOnFbo(fbo);
  }
}

ErrorCode ObstacleHandler::initInnerObstacles(
    const ObstacleHandlerConfig &cfg, const FieldDescription &fieldDescr,
    const std::vector<FieldPos> &obstaclePositions,
    const ObstacleOutInterface &obstacleOutInterface) {
  const size_t obstaclesCount = obstaclePositions.size();
  _innerObstacles.resize(obstaclesCount);

  constexpr double bigObjApproachOverlayScaleFactor = 1.5;
  constexpr double bigObstacleToTileRatio = 0.5;
  const int32_t bigOffsetFromTileX = static_cast<int32_t>(0.25
      * fieldDescr.tileWidth);
  const int32_t bigOffsetFromTileY = static_cast<int32_t>(0.25
      * fieldDescr.tileHeight);

  constexpr double smallObjApproachOverlayScaleFactor = 1.75;
  constexpr double smallObstacleToTileRatio = 0.35;
  const int32_t smallOffsetFromTileX = static_cast<int32_t>(0.33
      * fieldDescr.tileWidth);
  const int32_t smallOffsetFromTileY = static_cast<int32_t>(0.33
      * fieldDescr.tileHeight);

  ObstacleConfig obstacleCfg;
  obstacleCfg.status = cfg.status;
  obstacleCfg.rsrcId = cfg.obstacleRsrcId;
  obstacleCfg.tileWidth = fieldDescr.tileWidth;
  obstacleCfg.tileHeight = fieldDescr.tileHeight;
  obstacleCfg.obstacleVisibility = ObstacleVisibility::DEFAULT;

  for (size_t i = 0; i < obstaclesCount; ++i) {
    obstacleCfg.fieldPos = obstaclePositions[i];
    const char marker =
        fieldDescr.data[obstacleCfg.fieldPos.row][obstacleCfg.fieldPos.col];
    if (RoboCommonDefines::BIG_OBSTACLE_MARKER == marker) {
      obstacleCfg.width = bigObstacleToTileRatio * fieldDescr.tileWidth;
      obstacleCfg.height = bigObstacleToTileRatio * fieldDescr.tileHeight;
      obstacleCfg.tileOffset = Point(bigOffsetFromTileX, bigOffsetFromTileY);
      obstacleCfg.objApproachOverlayScaleFactor =
          bigObjApproachOverlayScaleFactor;
    } else {
      obstacleCfg.width = smallObstacleToTileRatio * fieldDescr.tileWidth;
      obstacleCfg.height = smallObstacleToTileRatio * fieldDescr.tileHeight;
      obstacleCfg.tileOffset = Point(smallOffsetFromTileX,
          smallOffsetFromTileY);
      obstacleCfg.objApproachOverlayScaleFactor =
          smallObjApproachOverlayScaleFactor;
    }

    if (ErrorCode::SUCCESS != _innerObstacles[i].init(obstacleCfg, fieldDescr,
        obstacleOutInterface)) {
      LOGERR("Error, _innerObstacles[%zu].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

ErrorCode ObstacleHandler::initOuterObstacles(
    const ObstacleHandlerConfig &cfg, const FieldDescription &fieldDescr,
    const ObstacleOutInterface &obstacleOutInterface) {
  constexpr double obstacleToTileRatio = 0.5;
  const int32_t offsetFromTileX = static_cast<int32_t>(0.25
      * fieldDescr.tileWidth);
  const int32_t offsetFromTileY = static_cast<int32_t>(0.25
      * fieldDescr.tileHeight);

  ObstacleConfig obstacleCfg;
  obstacleCfg.status = cfg.status;
  obstacleCfg.rsrcId = cfg.obstacleRsrcId;
  obstacleCfg.tileOffset = Point(offsetFromTileX, offsetFromTileY);
  obstacleCfg.objApproachOverlayScaleFactor = 1.5;
  obstacleCfg.tileWidth = fieldDescr.tileWidth;
  obstacleCfg.tileHeight = fieldDescr.tileHeight;
  obstacleCfg.width = obstacleToTileRatio * fieldDescr.tileWidth;
  obstacleCfg.height = obstacleToTileRatio * fieldDescr.tileHeight;
  obstacleCfg.obstacleVisibility = ObstacleVisibility::HIDDEN;

  const int32_t outerObstaclesCount = (fieldDescr.rows * 2)
      + (fieldDescr.cols * 2);
  _outerObstacles.resize(outerObstaclesCount);

  int32_t outerObstacleId = 0;

  //place obstacles 1 row above the first and 1 row after the last
  const std::array<int32_t, 2> rowIndexes { -1, fieldDescr.rows };
  for (const int32_t rowIdx : rowIndexes) {
    for (int32_t col = 0; col < fieldDescr.cols; ++col) {
      obstacleCfg.fieldPos.row = rowIdx;
      obstacleCfg.fieldPos.col = col;

      if (ErrorCode::SUCCESS != _outerObstacles[outerObstacleId].init(
              obstacleCfg, fieldDescr, obstacleOutInterface)) {
        LOGERR("Error, _outerObstacles[%d].init() failed", outerObstacleId);
        return ErrorCode::FAILURE;
      }

      ++outerObstacleId;
    }
  }

  //place obstacles 1 col left from the first and 1 col to the right of the last
  const std::array<int32_t, 2> colIndexes { -1, fieldDescr.cols };
  for (const int32_t colIdx : colIndexes) {
    for (int32_t row = 0; row < fieldDescr.rows; ++row) {
      obstacleCfg.fieldPos.row = row;
      obstacleCfg.fieldPos.col = colIdx;

      if (ErrorCode::SUCCESS != _outerObstacles[outerObstacleId].init(
              obstacleCfg, fieldDescr, obstacleOutInterface)) {
        LOGERR("Error, _outerObstacles[%d].init() failed", outerObstacleId);
        return ErrorCode::FAILURE;
      }

      ++outerObstacleId;
    }
  }

  return ErrorCode::SUCCESS;
}

