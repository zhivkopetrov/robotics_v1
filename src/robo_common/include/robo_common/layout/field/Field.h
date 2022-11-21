#ifndef ROBO_COMMON_FIELD_H_
#define ROBO_COMMON_FIELD_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/Tile.h"
#include "robo_common/layout/field/FieldPos.h"
#include "robo_common/layout/field/obstacles/ObstacleHandler.h"

//Forward declarations
class CollisionWatcher;
struct FieldConfig;

struct FieldOutInterface {
  ObjechApproachOverlayTriggeredCb objechApproachOverlayTriggeredCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

class Field {
public:
  ErrorCode init(const FieldConfig &cfg, const FieldOutInterface& interface);

  void draw() const;

  void updateFieldFbo();

  void setFieldDataMarker(const FieldPos &fieldPos, char fieldMarker);
  void resetFieldDataMarker(const FieldPos &fieldPos);

  const FieldDescription& getDescription() const;

  void toggleDebugTexts();

private:
  ErrorCode initTiles(const FieldConfig &cfg,
                      const FieldOutInterface& interface);

  //for debug purposes
  void printFieldData() const;

  Fbo _fieldFbo;
  std::vector<Tile> _tiles; //2D matrix tile layout as 1D representation
  FieldDescription _description;
  ObstacleHandler _obstacleHandler;
  FboOptimization _fboOptimization = FboOptimization::ENABLED;
};

#endif /* ROBO_COMMON_FIELD_H_ */
