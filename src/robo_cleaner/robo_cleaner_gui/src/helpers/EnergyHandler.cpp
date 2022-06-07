//Corresponding header
#include "robo_cleaner_gui/helpers/EnergyHandler.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/helpers/config/EnergyHandlerConfig.h"

ErrorCode EnergyHandler::init(const EnergyHandlerConfig& cfg) {
  _batteryStatus.maxMovesOnFullEnergy = cfg.maxMovesOnFullEnergy;
  _batteryStatus.movesLeft = cfg.maxMovesOnFullEnergy;
  _energyPerMove = INDICATOR_PANEL_MAX_VALUE / cfg.maxMovesOnFullEnergy;

  return ErrorCode::SUCCESS;
}

ErrorCode EnergyHandler::initiateMove() {
  if (0 <= _batteryStatus.movesLeft) {
    return ErrorCode::FAILURE;
  }

  --_batteryStatus.movesLeft;
  return ErrorCode::SUCCESS;
}

BatteryStatus EnergyHandler::charge() {
  ++_batteryStatus.movesLeft;
  if (_batteryStatus.movesLeft > _batteryStatus.maxMovesOnFullEnergy) {
    _batteryStatus.movesLeft = _batteryStatus.maxMovesOnFullEnergy;
  }
  return _batteryStatus;
}

BatteryStatus EnergyHandler::queryBatteryStatus() const {
  return _batteryStatus;
}
