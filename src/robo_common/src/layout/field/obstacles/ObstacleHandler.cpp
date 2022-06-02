//Corresponding header
#include "robo_common/layout/field/obstacles/ObstacleHandler.h"

//System headers

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/field/FieldUtils.h"
#include "robo_common/layout/field/config/ObstacleHandlerConfig.h"

ErrorCode ObstacleHandler::init(const ObstacleHandlerConfig &cfg,
                                const FieldDescription &fieldDescr,
                                const std::vector<FieldPos> &obstaclePositions,
                                const ObstacleHandlerOutInterface &interface) {
  if (nullptr == interface.collisionWatcher) {
    LOGERR("Error, nullptr provided for CollisionWatcher");
    return ErrorCode::FAILURE;
  }

  const size_t obstaclesCount = obstaclePositions.size();
  _obstacles.resize(obstaclesCount);

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

    if (ErrorCode::SUCCESS != _obstacles[i].init(obstacleCfg, fieldDescr,
            interface.collisionWatcher,
            interface.objechApproachOverlayTriggeredCb)) {
      LOGERR("Error, _obstacles[%zu].init() failed", i);
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

void ObstacleHandler::drawOnFbo(Fbo &fbo) const {
  for (const auto &obstacle : _obstacles) {
    obstacle.drawOnFbo(fbo);
  }
}

