#ifndef ROBO_CLEANER_GUI_RUBBISH_H_
#define ROBO_CLEANER_GUI_RUBBISH_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"
#include "robo_common/layout/field/ObjectApproachOverlay.h"
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class Fbo;

struct RubbishConfig {
  uint64_t rsrcId = 0;
  uint64_t counterTextFontId = 0;
  FieldPos fieldPos;
  Point tileOffset;
  int32_t width = 0;
  int32_t height = 0;
  int32_t textCounterValue = 0;
  double objApproachOverlayScaleFactor { };
};

struct RubbishOutInterface {
  ObjechApproachOverlayTriggeredCb objectApproachOverlayTriggeredCb;
  ContainerRedrawCb containerRedrawCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

class Rubbish {
public:
  ErrorCode init(const RubbishConfig &cfg, const RubbishOutInterface &interface,
                 const FieldDescription &fieldDescr);

  void drawOnFbo(Fbo& fbo) const;

  void modifyRubbishWidget(char fieldMarker);

private:
  ErrorCode createImage(const RubbishConfig &cfg,
                        const RubbishOutInterface &interface,
                        const FieldDescription &fieldDescr);

  ErrorCode createObjOverlay(const RubbishConfig &cfg,
                             const RubbishOutInterface &interface,
                             const FieldDescription &fieldDescr);

  void createText(const RubbishConfig &cfg, const FieldDescription &fieldDescr);

  void setImageFrame(int32_t counterValue);

  Image _img;
  Text _counterText;
  ObjectApproachOverlay _objApproachOverlay;
};

#endif /* ROBO_CLEANER_GUI_RUBBISH_H_ */
