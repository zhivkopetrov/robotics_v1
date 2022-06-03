#ifndef ROBO_CLEANER_GUI_ENTITYHANDLER_H_
#define ROBO_CLEANER_GUI_ENTITYHANDLER_H_

//System headers
#include <cstdint>
#include <vector>
#include <unordered_map>

//Other libraries headers
#include "manager_utils/drawing/Text.h"

//Own components headers
#include "robo_cleaner_gui/layout/entities/Rubbish.h"

//Forward declarations
struct EntityHandlerConfig;

class EntityHandler {
public:
  ErrorCode init(const EntityHandlerConfig &cfg,
                 const FieldDescription &fieldDescr);

  void draw() const;

private:
  std::vector<Rubbish> _rubbish;

  //key = (currRow * maxCols) + currCol
  //value = relative rubbish id
  std::unordered_map<int32_t, int32_t> _fieldPosToRubbishIdMapping;
};

#endif /* ROBO_CLEANER_GUI_ENTITYHANDLER_H_ */
