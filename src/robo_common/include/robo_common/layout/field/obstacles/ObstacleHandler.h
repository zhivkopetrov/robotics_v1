#ifndef ROBO_COMMON_OBSTACLEHANDLER_H_
#define ROBO_COMMON_OBSTACLEHANDLER_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/layout/field/obstacles/Obstacle.h"

//Forward declarations
class CollisionWatcher;
struct ObstacleHandlerConfig;

struct ObstacleHandlerOutInterface {
  ObjechApproachOverlayTriggeredCb objechApproachOverlayTriggeredCb;
  ContainerRedrawCb containerRedrawCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

class ObstacleHandler {
public:
  ErrorCode init(const ObstacleHandlerConfig &cfg,
                 const FieldDescription &fieldDescr,
                 const std::vector<FieldPos> &innerObstaclePositions,
                 const ObstacleHandlerOutInterface &interface);

  void draw() const;

  void drawOnFbo(Fbo &fbo) const;

private:
  ErrorCode initInnerObstacles(
      const ObstacleHandlerConfig &cfg, const FieldDescription &fieldDescr,
      const std::vector<FieldPos> &obstaclePositions,
      const ObstacleOutInterface &obstacleOutInterface);

  ErrorCode initOuterObstacles(
      const ObstacleHandlerConfig &cfg, const FieldDescription &fieldDescr,
      const ObstacleOutInterface &obstacleOutInterface);

  std::vector<Obstacle> _innerObstacles;
  std::vector<Obstacle> _outerObstacles;
};

#endif /* ROBO_COMMON_OBSTACLEHANDLER_H_ */
