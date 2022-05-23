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
struct ObstacleHandlerConfig;

class ObstacleHandler {
public:
  ErrorCode init(const ObstacleHandlerConfig &cfg,
                 const FieldDescription &fieldDescr,
                 const std::vector<FieldPos> &obstaclePositions);

  void drawOnFbo(Fbo &fbo) const;

private:
  std::vector<Obstacle> _obstacles;
};

#endif /* ROBO_COMMON_OBSTACLEHANDLER_H_ */
