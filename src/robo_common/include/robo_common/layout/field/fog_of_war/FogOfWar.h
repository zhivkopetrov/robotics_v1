#ifndef ROBO_COMMON_FOGOFWAR_H_
#define ROBO_COMMON_FOGOFWAR_H_

//System headers
#include <cstdint>
#include <unordered_map>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/layout/field/config/FogOfWarConfig.h"
#include "robo_common/layout/field/fog_of_war/FogCollisionObject.h"

//Forward declarations
class CollisionWatcher;
struct FieldConfig;

struct FogOfWarOutInterface {
  CollisionWatcher *collisionWatcher = nullptr;
};

class FogOfWar {
public:
  ErrorCode init(const FogOfWarConfig &fogCfg,
                 const FogOfWarOutInterface &outInterface,
                 const FieldConfig &fieldCfg);

  void draw() const;

  void revealAllFogTiles();

private:
  ErrorCode createFogTiles(const FieldConfig &fieldCfg,
                           const std::vector<int> &fogTilesFadeAnimTimerIds);

  ErrorCode populateFogTiles(uint64_t cloudRsrcId);

  void onFogObjectAnimComplete(int32_t id);

  std::unordered_map<int32_t, FogCollisionObject> _fogTiles;
  FogOfWarOutInterface _outInterface;
  FogOfWarStatus _status = FogOfWarStatus::DISABLED;
};

#endif /* ROBO_COMMON_FOGOFWAR_H_ */
