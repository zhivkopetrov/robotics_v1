//Corresponding header
#include "robo_cleaner_gui/helpers/EnergyHandler.h"

//System headers

//Other libraries headers
#include "robo_cleaner_common/defines/RoboCleanerDefines.h"
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/helpers/config/EnergyHandlerConfig.h"

ErrorCode EnergyHandler::init(const EnergyHandlerConfig& cfg,
                              const ModifyEnergyLevelCb& modifyEnergyLevelCb) {
  if (nullptr == modifyEnergyLevelCb) {
    LOGERR("Error, nullptr provided for ModifyEnergyLevelCb");
    return ErrorCode::FAILURE;
  }
  _modifyEnergyLevelCb = modifyEnergyLevelCb;

  _batteryStatus.maxMovesOnFullEnergy = cfg.maxMovesOnFullEnergy;
  _batteryStatus.movesLeft = cfg.maxMovesOnFullEnergy;
  _energyConsumedPerMove = INDICATOR_PANEL_MAX_VALUE / cfg.maxMovesOnFullEnergy;

  return ErrorCode::SUCCESS;
}

EnergyHandlerMoveOutcome EnergyHandler::initiateMove() {
  EnergyHandlerMoveOutcome outcome;
  if (0 >= _batteryStatus.movesLeft) {
    outcome.success = false;
    outcome.penaltyTurns = _batteryStatus.maxMovesOnFullEnergy;
    return outcome;
  }
  --_batteryStatus.movesLeft;

  _modifyEnergyLevelCb(-1 * _energyConsumedPerMove);
  return outcome;
}

BatteryStatus EnergyHandler::charge() {
  int32_t movesToCharge = RoboCleanerDefines::MOVES_REGAINED_ON_CHARGE_TURN;
  _batteryStatus.movesLeft += movesToCharge;
  if (_batteryStatus.movesLeft > _batteryStatus.maxMovesOnFullEnergy) {
    movesToCharge -=
        (_batteryStatus.movesLeft - _batteryStatus.maxMovesOnFullEnergy);
    _batteryStatus.movesLeft = _batteryStatus.maxMovesOnFullEnergy;
  }

  _modifyEnergyLevelCb(movesToCharge * _energyConsumedPerMove);
  return _batteryStatus;
}

BatteryStatus EnergyHandler::queryBatteryStatus() const {
  return _batteryStatus;
}

void EnergyHandler::performPenaltyChange() {
  _batteryStatus.movesLeft = _batteryStatus.maxMovesOnFullEnergy / 2;
  _modifyEnergyLevelCb(_batteryStatus.movesLeft * _energyConsumedPerMove);
}
