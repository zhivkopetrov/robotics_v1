//Corresponding header
#include "robo_miner_gui/helpers/MovementWatcher.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t MovementWatcher::init(
    const GetPlayerSurroundingTilesCb& getPlayerSurroundingTilesCb) {
  if (nullptr == getPlayerSurroundingTilesCb) {
    LOGERR("Error, nullptr provided for GetPlayerSurroundingTilesCb");
    return FAILURE;
  }
  _getPlayerSurroundingTilesCb = getPlayerSurroundingTilesCb;

  return SUCCESS;
}

bool MovementWatcher::waitForChange(const std::chrono::milliseconds &timeout,
                                    MoveOutcome &outOutcome,
                                    SurroundingTiles &outSurroundingTiles) {
  _ready = false;

  bool timedOut = true;
  std::unique_lock<std::mutex> uniqueLock(_mutex);
  timedOut = !_condVar.wait_for(uniqueLock, timeout, [this]() {
    return _ready;
  });

  if (timedOut) {
    LOGR("MovementWatcher::waitForChange() timed out after %ld ms",
        timeout.count());
    return false;
  }

  outOutcome = _outcome;
  outSurroundingTiles = _playerSurroundingTiles;
  return true;
}

void MovementWatcher::changeState(MoveOutcome outcome) {
  std::lock_guard<std::mutex> lockGuard(_mutex);
  _ready = true;
  _outcome = outcome;
  _playerSurroundingTiles = _getPlayerSurroundingTilesCb();
  _condVar.notify_one();
}
