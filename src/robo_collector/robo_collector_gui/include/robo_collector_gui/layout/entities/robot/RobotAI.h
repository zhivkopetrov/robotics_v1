#ifndef ROBO_COLLECTOR_GUI_ROBOTAI_H_
#define ROBO_COLLECTOR_GUI_ROBOTAI_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Forward declarations
struct RobotActInterface;

struct RobotAIConfig {
  GetFieldDescriptionCb getFieldDescriptionCb;
  char fieldEnemyMarker = '!';
};

class RobotAI {
public:
  int32_t init(const RobotAIConfig &cfg);

  void makeMove(const RobotActInterface &actInterface);

private:
  bool isForwardDirValid(const FieldPos &currFieldPos, Direction currDir) const;

  GetFieldDescriptionCb _getFieldDescriptionCb;
  char _fieldEnemyMarker = '!';
};

#endif /* ROBO_COLLECTOR_GUI_ROBOTAI_H_ */
