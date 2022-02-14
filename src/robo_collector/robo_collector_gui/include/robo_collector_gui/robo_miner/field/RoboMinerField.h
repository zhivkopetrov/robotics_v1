#ifndef ROBO_MINER_ROBOMINERFIELD_H_
#define ROBO_MINER_ROBOMINERFIELD_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "robo_common/defines/RoboCommonDefines.h"

//Own components headers

//Forward declarations
struct RoboMinerFieldConfig;

class RoboMinerField {
public:
  int32_t init(const RoboMinerFieldConfig &cfg);

  const FieldData& getFieldData() const;
  char getEmptyMarker() const;

private:
  FieldData _fieldData;
  char _emptyDataMarker = '.';
};

#endif /* ROBO_MINER_ROBOMINERFIELD_H_ */
