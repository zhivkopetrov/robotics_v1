//Corresponding header
#include "robo_cleaner_gui/helpers/EnergyHandler.h"

//System headers

//Other libraries headers
#include "robo_cleaner_common/defines/RoboCleanerDefines.h"
#include "utils/log/Log.h"

//Own components headers
#include "robo_cleaner_gui/helpers/config/EnergyHandlerConfig.h"

ErrorCode EnergyHandler::init(const EnergyHandlerConfig &cfg,
                              const ModifyEnergyLevelCb &modifyEnergyLevelCb) {
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

    --_allowedPenaltyMoves;
    LOGR("Insufficient energy to perform a movement. "
         "Applying penalty. Allowed penalties left: %d", _allowedPenaltyMoves);

    if (0 >= _allowedPenaltyMoves) {
      outcome.majorError = true;
    }

    return outcome;
  }
  --_batteryStatus.movesLeft;

  _modifyEnergyLevelCb(-1 * _energyConsumedPerMove);
  return outcome;
}

ChargeOutcome EnergyHandler::charge(ChargeDuration duration,
                                    int32_t chargeTurns) {
  ChargeOutcome outcome;
  if (0 > chargeTurns) {
    outcome.success = false;
    outcome.errorReason = "Negative value: (";
    outcome.errorReason.append(std::to_string(chargeTurns)).append(
        ") provided for chargeTurns");
    return outcome;
  }

  if (ChargeDuration::UNTIL_FULL == duration) {
    BatteryStatus status;
    while (true) {
      status = chargeOnce();
      ++outcome.turnsSpendCharging;
      if (status.maxMovesOnFullEnergy == status.movesLeft) {
        outcome.batteryStatus = status;
        return outcome;
      }
    }
  }

  outcome.turnsSpendCharging = chargeTurns;
  for (int32_t i = 0; i < chargeTurns; ++i) {
    chargeOnce();
  }
  outcome.batteryStatus = _batteryStatus;

  return outcome;
}

BatteryStatus EnergyHandler::queryBatteryStatus() const {
  return _batteryStatus;
}

void EnergyHandler::performPenaltyChange() {
  _batteryStatus.movesLeft = _batteryStatus.maxMovesOnFullEnergy / 2;
  _modifyEnergyLevelCb(_batteryStatus.movesLeft * _energyConsumedPerMove);
}

BatteryStatus EnergyHandler::chargeOnce() {
  int32_t movesToCharge = RoboCleanerDefines::MOVES_REGAINED_ON_CHARGE_TURN;
  _batteryStatus.movesLeft += movesToCharge;
  if (_batteryStatus.movesLeft > _batteryStatus.maxMovesOnFullEnergy) {
    movesToCharge -= (_batteryStatus.movesLeft
        - _batteryStatus.maxMovesOnFullEnergy);
    _batteryStatus.movesLeft = _batteryStatus.maxMovesOnFullEnergy;
  }

  const int32_t modifyEnergyValue = movesToCharge * _energyConsumedPerMove;
  if (0 != modifyEnergyValue) {
    _modifyEnergyLevelCb(modifyEnergyValue);
  }

  return _batteryStatus;
}
