#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMINITHELPER_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMINITHELPER_H_

//System headers
#include <cstdint>
#include <any>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class UrControlBloom;
struct UrControlCommonLayoutConfig;
struct UrContolBloomExternalBridgeConfig;
struct UrControlCommonLayoutInterface;

class UrControlBloomInitHelper {
public:
  UrControlBloomInitHelper() = delete;

  static ErrorCode init(const std::any &cfg, UrControlBloom &Bloom);

private:
  static ErrorCode initLayout(
      const UrControlCommonLayoutConfig &cfg,
      UrControlCommonLayoutInterface &layoutInterface, //out param
      UrControlBloom &gui);

  static ErrorCode initDashboardHelper(
      const UrControlCommonLayoutInterface &layoutInterface, 
      UrControlBloom &gui);

  static ErrorCode initUrControlBloomExternalBridge(
      const UrContolBloomExternalBridgeConfig &cfg,
      const UrControlCommonLayoutInterface &layoutInterface, 
      UrControlBloom &gui);
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMINITHELPER_H_ */
