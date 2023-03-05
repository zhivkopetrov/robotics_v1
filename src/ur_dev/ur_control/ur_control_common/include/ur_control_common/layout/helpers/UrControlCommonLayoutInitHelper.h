#ifndef UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTINITHELPER_H_
#define UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTINITHELPER_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class UrControlCommonLayout;
struct UrControlCommonLayoutConfig;
struct UrControlCommonLayoutOutInterface;
struct ButtonHandlerHighLevelConfig;

class UrControlCommonLayoutInitHelper {
public:
  UrControlCommonLayoutInitHelper() = delete;

  static ErrorCode init(const UrControlCommonLayoutConfig &cfg,
                        const UrControlCommonLayoutOutInterface &outInterface,
                        UrControlCommonLayout &layout);

private:
  static ErrorCode initStandaloneImages(const UrControlCommonLayoutConfig &cfg,
                                        UrControlCommonLayout &layout);

  static ErrorCode initButtonHandler(
      const ButtonHandlerHighLevelConfig &cfg,
      const UrControlCommonLayoutOutInterface &outInterface,
      UrControlCommonLayout &layout);
};

#endif /* UR_CONTROL_COMMON_URCONTROLCOMMONLAYOUTINITHELPER_H_ */
