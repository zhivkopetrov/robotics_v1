#ifndef ROBO_CLEANER_GUI_ENTITYHANDLER_H_
#define ROBO_CLEANER_GUI_ENTITYHANDLER_H_

//System headers
#include <cstdint>
#include <vector>
#include <unordered_map>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"

//Own components headers
#include "robo_cleaner_gui/layout/entities/Rubbish.h"

//Forward declarations
struct EntityHandlerConfig;

struct EntityHandlerOutInterface {
  ObjechApproachOverlayTriggeredCb objectApproachOverlayTriggeredCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

class EntityHandler {
public:
  ErrorCode init(const EntityHandlerConfig &cfg,
                 const EntityHandlerOutInterface& interface,
                 const FieldDescription &fieldDescr);

  void draw() const;

  void modifyRubbishWidget(const FieldPos& fieldPos, char fieldMarker);

private:
  void createEntityHandlerFbo(const FieldDescription &fieldDescr);

  ErrorCode createRubbishEntities(const EntityHandlerConfig &cfg,
                                  const EntityHandlerOutInterface& interface,
                                  const FieldDescription &fieldDescr);

  void createChargingStation(uint64_t rsrcId, const FieldPos& fieldPos,
                             const FieldDescription &fieldDescr);

  void updateEntityHandlerFbo();

  Image _chargingStationImg;

  std::vector<Rubbish> _rubbish;

  //key = (currRow * maxCols) + currCol
  //value = relative rubbish id
  std::unordered_map<int32_t, int32_t> _fieldPosToRubbishIdMapping;

  int32_t _fieldCols { };

  Fbo _fbo;
};

#endif /* ROBO_CLEANER_GUI_ENTITYHANDLER_H_ */
