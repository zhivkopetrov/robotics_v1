//Corresponding header
#include "robo_miner_gui/helpers/MovementWatcher.h"

//System headers

//Other libraries headers
#include "utils/log/Log.h"

//Own components headers

ErrorCode MovementWatcher::init(
    const GetPlayerSurroundingTilesCb &getPlayerSurroundingTilesCb) {
  if (nullptr == getPlayerSurroundingTilesCb) {
    LOGERR("Error, nullptr provided for GetPlayerSurroundingTilesCb");
    return ErrorCode::FAILURE;
  }
  _getPlayerSurroundingTilesCb = getPlayerSurroundingTilesCb;

  return ErrorCode::SUCCESS;
}

bool MovementWatcher::waitForChange(const std::chrono::milliseconds &timeout,
                                    MovementWatchOutcome &outcome) {
  bool timedOut = true;
  std::unique_lock<std::mutex> uniqueLock(_mutex);
  _ready = false;
  timedOut = !_condVar.wait_for(uniqueLock, timeout, [this]() {
    return _ready;
  });

  if (timedOut) {
    LOGR("MovementWatcher::waitForChange() timed out after %ld ms",
        timeout.count());
    return false;
  }

  outcome = _lastOutcome;
  return true;
}

void MovementWatcher::changeState(const RobotState &state,
                                  MoveOutcome outcome) {
  std::unique_lock<std::mutex> lock(_mutex);
  _ready = true;
  _lastOutcome.moveOutcome = outcome;
  _lastOutcome.robotPos = state.fieldPos;
  _lastOutcome.robotDir = state.dir;
  _lastOutcome.surroundingTiles = _getPlayerSurroundingTilesCb();
  _lastOutcome.actionTerminated = false;

  lock.unlock();
  _condVar.notify_one();
}

void MovementWatcher::terminateAction() {
  std::unique_lock<std::mutex> lock(_mutex);
  _ready = true;
  _lastOutcome.actionTerminated = true;

  lock.unlock();
  _condVar.notify_one();
}
