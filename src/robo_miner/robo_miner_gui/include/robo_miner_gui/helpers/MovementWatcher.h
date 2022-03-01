#ifndef ROBO_MINER_GUI_MOVEMENTWATCHER_H_
#define ROBO_MINER_GUI_MOVEMENTWATCHER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <condition_variable>
#include <mutex>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"

//Own components headers

//Forward declarations

struct MovementWatchOutcome {
  SurroundingTiles surroundingTiles;
  FieldPos robotPos;
  MoveOutcome moveOutcome;
};

class MovementWatcher: public NonCopyable, public NonMoveable {
public:
  int32_t init(const GetPlayerSurroundingTilesCb &getPlayerSurroundingTilesCb);

  //returns success or not if timed out
  bool waitForChange(const std::chrono::milliseconds &timeout,
                     MovementWatchOutcome &outcome);

  void changeState(const RobotState& state, MoveOutcome outcome);

private:
  std::mutex _mutex;
  std::condition_variable _condVar;

  GetPlayerSurroundingTilesCb _getPlayerSurroundingTilesCb;
  MovementWatchOutcome _lastOutcome;
  bool _ready = false;
};

#endif /* ROBO_MINER_GUI_MOVEMENTWATCHER_H_ */
