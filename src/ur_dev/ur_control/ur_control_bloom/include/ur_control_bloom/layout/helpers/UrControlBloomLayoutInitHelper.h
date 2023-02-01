#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUTINITHELPER_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUTINITHELPER_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class UrControlBloomLayout;
struct UrControlBloomLayoutConfig;
struct UrControlCommonLayoutOutInterface;
struct UrControlCommonLayoutInterface;

class UrControlBloomLayoutInitHelper {
public:
  UrControlBloomLayoutInitHelper() = delete;

  static ErrorCode init(
    const UrControlBloomLayoutConfig &cfg,
    const UrControlCommonLayoutOutInterface& commonOutInterface,
    UrControlCommonLayoutInterface &commonInterface,
    UrControlBloomLayout &layout);

private:
  static ErrorCode initStandaloneImages(const UrControlBloomLayoutConfig &cfg,
                                        UrControlBloomLayout &layout);
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMLAYOUTINITHELPER_H_ */
