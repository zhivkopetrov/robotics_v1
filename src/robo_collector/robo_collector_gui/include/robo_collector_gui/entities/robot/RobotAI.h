#ifndef ROBO_COLLECTOR_GUI_ROBOTAI_H_
#define ROBO_COLLECTOR_GUI_ROBOTAI_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Forward declarations

struct RobotActInterface {
  RobotActInterface(const RobotActCb inputActCb,
                    GetRobotFieldPosCb inputGetFieldPosCb,
                    GetRobotDirCb inputGetDirCb)
      : actCb(inputActCb), getFieldPosCb(inputGetFieldPosCb),
        getDirCb(inputGetDirCb) {
  }

  RobotActCb actCb;
  GetRobotFieldPosCb getFieldPosCb;
  GetRobotDirCb getDirCb;
};

struct RobotAIConfig {
  GetFieldDataCb getFieldDataCb;
  char fieldEnemyMarker = '!';
};

class RobotAI {
public:
  int32_t init(const RobotAIConfig &cfg);

  void makeMove(const RobotActInterface &actInterface);

private:
  bool isForwardDirValid(const FieldPos &currFieldPos, Direction currDir) const;

  GetFieldDataCb _getFieldDataCb;
  char _fieldEnemyMarker = '!';
};

#endif /* ROBO_COLLECTOR_GUI_ROBOTAI_H_ */
