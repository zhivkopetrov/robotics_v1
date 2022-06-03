#ifndef ROBO_CLEANER_GUI_RUBBISH_H_
#define ROBO_CLEANER_GUI_RUBBISH_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

struct RubbishConfig {
  uint64_t rsrcId = 0;
  uint64_t counterTextFontId = 0;
  FieldPos fieldPos;
  Point tileOffset;
  int32_t width = 0;
  int32_t height = 0;
  int32_t textCounterValue = 0;
};

class Rubbish {
public:
  ErrorCode init(const RubbishConfig &cfg, const FieldDescription &fieldDescr);
  void draw() const;

private:
  void createImage(const RubbishConfig &cfg,
                   const FieldDescription &fieldDescr);
  void createText(const RubbishConfig &cfg, const FieldDescription &fieldDescr);

  Image _img;
  Text _counterText;
};

#endif /* ROBO_CLEANER_GUI_RUBBISH_H_ */
