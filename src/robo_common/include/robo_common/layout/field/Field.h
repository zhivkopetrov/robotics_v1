#ifndef ROBO_COMMON_FIELD_H_
#define ROBO_COMMON_FIELD_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/Tile.h"
#include "robo_common/layout/field/FieldPos.h"

//Forward declarations
struct FieldConfig;

class Field {
public:
  int32_t init(const FieldConfig &cfg);

  void draw() const;

  void updateFieldFbo();

  void setFieldDataMarker(const FieldPos &fieldPos, char fieldMarker);
  void resetFieldDataMarker(const FieldPos &fieldPos);

  const FieldData& getFieldData() const;
  const FieldDescription& getDescription() const;

  void toggleDebugTexts();

private:
  int32_t initTiles(const FieldConfig &cfg);

  //for debug purposes
  void printFieldData() const;

  Fbo _fieldFbo;
  std::vector<Tile> _tiles; //2D matrix tile layout as 1D representation
  FieldDescription _description;
};

#endif /* ROBO_COMMON_FIELD_H_ */
