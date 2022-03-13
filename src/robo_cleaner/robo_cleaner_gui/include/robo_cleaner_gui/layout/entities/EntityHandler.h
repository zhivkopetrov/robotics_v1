#ifndef ROBO_CLEANER_GUI_ENTITYHANDLER_H_
#define ROBO_CLEANER_GUI_ENTITYHANDLER_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "manager_utils/drawing/Text.h"

//Own components headers
#include "robo_cleaner_gui/layout/entities/Rubbish.h"

//Forward declarations
struct EntityHandlerConfig;

class EntityHandler {
public:
  ErrorCode init(const EntityHandlerConfig& cfg,
                 const GetFieldDescriptionCb& getFieldDescriptionCb);
  void deinit();
  void draw() const;

private:
  void createCounterText(const FieldPos &fieldPos, uint64_t fontId,
                         int32_t counterValue);

  void createObstacle(const FieldPos &fieldPos, uint64_t rsrcId);

  std::vector<Rubbish> _rubbish;
  std::vector<Text> _tileCounters;
  std::vector<Image> _obstacles;

  GetFieldDescriptionCb _getFieldDescriptionCb;
};

#endif /* ROBO_CLEANER_GUI_ENTITYHANDLER_H_ */
