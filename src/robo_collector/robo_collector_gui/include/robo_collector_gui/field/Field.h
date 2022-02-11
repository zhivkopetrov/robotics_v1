#ifndef ROBO_COLLECTOR_GUI_FIELD_H_
#define ROBO_COLLECTOR_GUI_FIELD_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"
#include "robo_collector_gui/field/Tile.h"
#include "robo_collector_gui/field/FieldPos.h"

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

  void toggleDebugTexts();

private:
  //for debug purposes
  void printFieldData() const;

  Fbo _fieldFbo;
  std::vector<std::vector<Tile>> _tiles;
  FieldData _fieldData;
  char _emptyDataMarker = '.';
};

#endif /* ROBO_COLLECTOR_GUI_FIELD_H_ */
