#ifndef ROBO_COLLECTOR_GUI_ROBOTAI_H_
#define ROBO_COLLECTOR_GUI_ROBOTAI_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/ErrorCode.h"

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
  ErrorCode init(const RobotAIConfig &cfg);

  void makeMove(const RobotActInterface &actInterface);

private:
  bool isForwardDirValid(const RobotState& state) const;

  GetFieldDescriptionCb _getFieldDescriptionCb;
  char _fieldEnemyMarker = '!';
};

#endif /* ROBO_COLLECTOR_GUI_ROBOTAI_H_ */
