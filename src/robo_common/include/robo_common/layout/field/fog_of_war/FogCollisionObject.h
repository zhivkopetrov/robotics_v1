#ifndef ROBO_COMMON_FOGCOLLISIONOBJECT_H_
#define ROBO_COMMON_FOGCOLLISIONOBJECT_H_

//System headers
#include <cstdint>
#include <functional>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"
#include "manager_utils/time/TimerClient.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/helpers/CollisionObject.h"

//Forward declarations
class Image;
class CollisionWatcher;

using OnFogObjectAimCompleteCb = std::function<void(int)>;

class FogCollisionObject final : public CollisionObject,
    public Fbo,
    public TimerClient {
public:
  ErrorCode init(const Rectangle &boundary, int32_t id, int32_t timerId,
                 const OnFogObjectAimCompleteCb &onAnimCompleteCb,
                 CollisionWatcher *collisionWatcher);
  void populateVisualContent(const Image &fogImg);

  Rectangle getBoundary() const override;

private:
  void registerCollision(const Rectangle &intersectRect,
                         CollisionDamageImpact impact) override;

  void onTimeout(const int32_t timerId) override;

  OnFogObjectAimCompleteCb _onAnimCompleteCb;
  CollisionWatcher *_collisionWatcher = nullptr;
  int32_t _id { };
  int32_t _timerId { };
};

#endif /* ROBO_COMMON_FOGCOLLISIONOBJECT_H_ */
