#ifndef ROBO_COLLECTOR_GUI_FIELD_H_
#define ROBO_COLLECTOR_GUI_FIELD_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"

//Own components headers
#include "robo_collector_gui/field/Tile.h"

//Forward declarations
struct FieldConfig;

class Field {
public:
  int32_t init(const FieldConfig &cfg);

  void draw() const;

  void updateFieldFbo();

  std::vector<std::vector<Tile>> _tiles;

  Fbo _fieldFbo;
};

#endif /* ROBO_COLLECTOR_GUI_FIELD_H_ */
