#ifndef ROBO_COMMON_OBJECTAPPROACHOVERLAY_H_
#define ROBO_COMMON_OBJECTAPPROACHOVERLAY_H_

#define DEBUG_VISUAL_OVERLAY 0

//System headers
#include <cstdint>

//Other libraries headers
#if DEBUG_VISUAL_OVERLAY
#include "manager_utils/drawing/Fbo.h"
#endif //DEBUG_VISUAL_OVERLAY

#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/helpers/CollisionObject.h"
#include "robo_common/layout/field/FieldPos.h"

//Forward declarations
class CollisionWatcher;

struct ObjectApproachOverlayConfig {
  Rectangle preScaledOverlayBoundary;
  Rectangle upperBoundary;
  double scaleFactor { };
  FieldPos fieldPos;
  bool isInsideField = true; //map surrounding obstacles are outside of field
  CollisionWatcher *collisionWatcher = nullptr;
};

class ObjectApproachOverlay final : public CollisionObject {
public:
  ErrorCode init(
      const ObjectApproachOverlayConfig &cfg,
      const ObjechApproachOverlayTriggeredCb &objechApproachOverlayTriggeredCb);

  void deactivate();

  void changeBoundary(const Rectangle &preScaledBoundary);

#if DEBUG_VISUAL_OVERLAY
  void drawOnFbo(Fbo &fbo) const;
#endif //DEBUG_VISUAL_OVERLAY

private:
  void registerCollision(const Rectangle &intersectRect,
                         CollisionDamageImpact impact) override;

  Rectangle getBoundary() const override;

  void alignWidget();

  ObjectApproachOverlayConfig _cfg;
  Rectangle _boundary;
  ObjechApproachOverlayTriggeredCb _objechApproachOverlayTriggeredCb;

#if DEBUG_VISUAL_OVERLAY
  Fbo _visualFbo;
#endif //DEBUG_VISUAL_OVERLAY
};

#endif /* ROBO_COMMON_OBJECTAPPROACHOVERLAY_H_ */
