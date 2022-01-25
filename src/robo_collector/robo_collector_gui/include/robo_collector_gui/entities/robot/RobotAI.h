#ifndef ROBO_COLLECTOR_GUI_ROBOTAI_H_
#define ROBO_COLLECTOR_GUI_ROBOTAI_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/defines/RoboCollectorGuiFunctionalDefines.h"

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
  char fieldEmptyDataMarker = '!';
  char playerDataMarker = '?';
};

class RobotAI {
public:
  int32_t init(const RobotAIConfig &cfg);

  void makeMove(const RobotActInterface &actInterface);

private:
  bool isForwardDirValid(const FieldPos &currFieldPos, Direction currDir) const;

  GetFieldDataCb _getFieldDataCb;
  char _fieldEmptyDataMarker = '!';
  char _playerDataMarker = '?';
};

#endif /* ROBO_COLLECTOR_GUI_ROBOTAI_H_ */
