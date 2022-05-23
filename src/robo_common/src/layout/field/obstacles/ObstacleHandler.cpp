//Corresponding header
#include "robo_common/layout/field/obstacles/ObstacleHandler.h"

//System headers

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"
#include "utils/Log.h"

//Own components headers
#include "robo_common/layout/field/FieldUtils.h"
#include "robo_common/layout/field/config/ObstacleHandlerConfig.h"

ErrorCode ObstacleHandler::init(
    const ObstacleHandlerConfig &cfg, const FieldDescription &fieldDescr,
    const std::vector<FieldPos> &obstaclePositions) {
  const size_t obstaclesCount = obstaclePositions.size();
  _obstacles.resize(obstaclesCount);

  constexpr double bigObstacleToTileRatio = 0.6;
  const int32_t bigOffsetFromTileX = static_cast<int32_t>(0.2
      * fieldDescr.tileWidth);
  const int32_t bigOffsetFromTileY = static_cast<int32_t>(0.2
      * fieldDescr.tileHeight);

  constexpr double smallObstacleToTileRatio = 0.35;
  const int32_t smallOffsetFromTileX = static_cast<int32_t>(0.33
      * fieldDescr.tileWidth);
  const int32_t smallOffsetFromTileY = static_cast<int32_t>(0.33
      * fieldDescr.tileHeight);

  ObstacleConfig obstacleCfg;
  obstacleCfg.rsrcId = cfg.obstacleRsrcId;

  for (size_t i = 0; i < obstaclesCount; ++i) {
    obstacleCfg.fieldPos = obstaclePositions[i];
    const char marker =
        fieldDescr.data[obstacleCfg.fieldPos.row][obstacleCfg.fieldPos.col];
    if (RoboCommonDefines::BIG_OBSTACLE_MARKER == marker) {
      obstacleCfg.width = bigObstacleToTileRatio * fieldDescr.tileWidth;
      obstacleCfg.height = bigObstacleToTileRatio * fieldDescr.tileHeight;
      obstacleCfg.tileOffset = Point(bigOffsetFromTileX, bigOffsetFromTileY);
    } else {
      obstacleCfg.width = smallObstacleToTileRatio * fieldDescr.tileWidth;
      obstacleCfg.height = smallObstacleToTileRatio * fieldDescr.tileHeight;
      obstacleCfg.tileOffset =
          Point(smallOffsetFromTileX, smallOffsetFromTileY);
    }

    if (ErrorCode::SUCCESS != _obstacles[i].init(obstacleCfg, fieldDescr)) {
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

