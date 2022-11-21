#ifndef ROBO_MINER_GUI_CRYSTALHANDLER_H_
#define ROBO_MINER_GUI_CRYSTALHANDLER_H_

//System headers
#include <cstdint>
#include <vector>
#include <unordered_map>

//Other libraries headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "manager_utils/drawing/Fbo.h"

//Own components headers
#include "robo_miner_gui/layout/entities/Crystal.h"

struct CrystalHandlerConfig {
  uint64_t crystalRsrcId = 0;
  GetFieldDescriptionCb getFieldDescriptionCb;
};

class CrystalHandler {
public:
  ErrorCode init(const CrystalHandlerConfig& cfg);
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent &e);

  void onCrystalClicked(const FieldPos& fieldPos);

private:
  ErrorCode initCrystals(const CrystalHandlerConfig& cfg);

  void createFbo();
  void updateFbo();

  std::vector<Crystal> _crystals;

  //key = (currRow * maxCols) + currCol
  //value = relative crystal id
  std::unordered_map<int32_t, int32_t> _fieldPosToCrystalIdMapping;

  Fbo _allCrystalsFbo;

  GetFieldDescriptionCb _getFieldDescriptionCb;
};

#endif /* ROBO_MINER_GUI_CRYSTALHANDLER_H_ */
