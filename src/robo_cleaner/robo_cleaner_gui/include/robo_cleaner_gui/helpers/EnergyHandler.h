#ifndef ROBO_CLEANER_GUI_ENERGYHANDLER_H_
#define ROBO_CLEANER_GUI_ENERGYHANDLER_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers

struct BatteryStatus {
  int32_t maxMovesOnFullEnergy { };
  int32_t movesLeft { };
};

//Forward declarations
struct EnergyHandlerConfig;

class EnergyHandler: public NonCopyable, public NonMoveable {
public:
  ErrorCode init(const EnergyHandlerConfig& cfg);

  ErrorCode initiateMove();

  BatteryStatus charge();

  BatteryStatus queryBatteryStatus() const;

private:
  BatteryStatus _batteryStatus;

  int32_t _energyPerMove { };
};

#endif /* ROBO_CLEANER_GUI_ENERGYHANDLER_H_ */
