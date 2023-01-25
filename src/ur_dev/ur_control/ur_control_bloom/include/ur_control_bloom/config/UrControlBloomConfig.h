#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMCONFIG_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMCONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "ur_control_common/layout/config/UrControlCommonLayoutConfig.h"

//Own components headers
#include "ur_control_bloom/external_api/config/UrContolBloomExternalBridgeConfig.h"

//Forward declarations

struct UrControlBloomConfig {
  UrControlCommonLayoutConfig commonLayoutCfg;
  UrContolBloomExternalBridgeConfig externalBridgeCfg;
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMCONFIG_H_ */

