#ifndef ROBO_CLEANER_GUI_ROBOCLEANERFIELD_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERFIELD_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "manager_utils/drawing/Text.h"

//Own components headers
#include "robo_cleaner_gui/layout/entities/Rubbish.h"

//Forward declarations
struct RoboCleanerFieldConfig;

class RoboCleanerField {
public:
  int32_t init(const RoboCleanerFieldConfig &cfg);
  void draw() const;

  const FieldData& getFieldData() const;
  char getEmptyMarker() const;

private:
  int32_t initEntities(const RoboCleanerFieldConfig &cfg);

  void createCounterText(const FieldPos &fieldPos, uint64_t fontId,
                         int32_t counterValue);

  void createObstacle(const FieldPos &fieldPos, uint64_t rsrcId);

  std::vector<Rubbish> _rubbish;
  std::vector<Text> _tileCounters;
  std::vector<Image> _obstacles;

  FieldData _fieldData;
  char _emptyDataMarker = '.';
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERFIELD_H_ */
