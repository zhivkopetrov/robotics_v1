#ifndef ROBO_CLEANER_GUI_ENERGYHANDLER_H_
#define ROBO_CLEANER_GUI_ENERGYHANDLER_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiFunctionalDefines.h"

struct BatteryStatus {
  int32_t maxMovesOnFullEnergy { };
  int32_t movesLeft { };
};

//Forward declarations
struct EnergyHandlerConfig;

struct EnergyHandlerMoveOutcome {
  bool success = true;
  int32_t penaltyTurns { };
};

struct ChargeOutcome {
  BatteryStatus batteryStatus;
  int32_t turnsSpendCharging {};
  std::string errorReason;
  bool success = true;
};

enum class ChargeDuration {
  TURN_BASED, UNTIL_FULL
};

class EnergyHandler: public NonCopyable, public NonMoveable {
public:
  ErrorCode init(const EnergyHandlerConfig& cfg,
                 const ModifyEnergyLevelCb& modifyEnergyLevelCb);

  EnergyHandlerMoveOutcome initiateMove();

  ChargeOutcome charge(ChargeDuration duration, int32_t chargeTurns);

  BatteryStatus queryBatteryStatus() const;

  void performPenaltyChange();

private:
  BatteryStatus chargeOnce();

  BatteryStatus _batteryStatus;

  int32_t _energyConsumedPerMove { };

  ModifyEnergyLevelCb _modifyEnergyLevelCb;
};

#endif /* ROBO_CLEANER_GUI_ENERGYHANDLER_H_ */
