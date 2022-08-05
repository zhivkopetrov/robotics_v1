#ifndef UR_CONTROL_GUI_URCONTROLGUILAYOUTINITHELPER_H_
#define UR_CONTROL_GUI_URCONTROLGUILAYOUTINITHELPER_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class UrControlGuiLayout;
struct UrControlGuiLayoutConfig;
struct UrControlGuiLayoutOutInterface;

class UrControlGuiLayoutInitHelper {
public:
  UrControlGuiLayoutInitHelper() = delete;

  static ErrorCode init(const UrControlGuiLayoutConfig &cfg,
                        const UrControlGuiLayoutOutInterface &outInterface,
                        UrControlGuiLayout &layout);

private:
  static ErrorCode initButtonHandler(
      const UrControlGuiLayoutConfig &cfg,
      const UrControlGuiLayoutOutInterface &outInterface,
      UrControlGuiLayout &layout);
};

#endif /* UR_CONTROL_GUI_URCONTROLGUILAYOUTINITHELPER_H_ */
