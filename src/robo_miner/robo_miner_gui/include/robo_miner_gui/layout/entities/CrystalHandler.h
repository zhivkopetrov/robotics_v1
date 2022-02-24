#ifndef ROBO_MINER_GUI_CRYSTALHANDLER_H_
#define ROBO_MINER_GUI_CRYSTALHANDLER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>
#include <unordered_map>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Own components headers
#include "robo_miner_gui/layout/entities/Crystal.h"

struct CrystalHandlerConfig {
  uint64_t crystalRsrcId = 0;
  GetFieldDescriptionCb getFieldDescriptionCb;
};

class CrystalHandler {
public:
  int32_t init(const CrystalHandlerConfig& cfg);
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent &e);

  void onCrystalClicked(const FieldPos& fieldPos);

private:
  int32_t initCrystals(const CrystalHandlerConfig& cfg);

  std::vector<Crystal> _crystals;

  //key = (currRow * maxCols) + currCol
  //value = relative crystal id
  std::unordered_map<int32_t, int32_t> _fieldPosToCrystalIdMapping;

  GetFieldDescriptionCb _getFieldDescriptionCb;
};

#endif /* ROBO_MINER_GUI_CRYSTALHANDLER_H_ */
