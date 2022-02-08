#ifndef ROBO_MINER_CRYSTAL_H_
#define ROBO_MINER_CRYSTAL_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/input/ButtonBase.h"

//Own components headers
#include "robo_collector_gui/field/FieldPos.h"
#include "robo_collector_gui/robo_miner/defines/RoboMinerDefines.h"

//Forward declarations
class InputEvent;

struct CrystalConfig {
  uint64_t rsrcId;
  FieldPos fieldPos;
  Point tileOffset;
  CrystalType type;
};

class Crystal final : public ButtonBase {
public:
  int32_t init(const CrystalConfig& cfg);
  void handleEvent(const InputEvent& e) override;

private:
  void onLeave(const InputEvent& e) override;
  void onReturn(const InputEvent& e) override;
};

#endif /* ROBO_MINER_CRYSTAL_H_ */
