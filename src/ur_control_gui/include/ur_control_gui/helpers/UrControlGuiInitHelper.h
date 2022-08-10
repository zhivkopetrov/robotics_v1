#ifndef UR_CONTROL_GUI_URCONTROLGUIINITHELPER_H_
#define UR_CONTROL_GUI_URCONTROLGUIINITHELPER_H_

//System headers
#include <cstdint>
#include <any>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class UrControlGui;
struct UrControlGuiLayoutConfig;
struct UrContolGuiExternalBridgeConfig;

class UrControlGuiInitHelper {
public:
  UrControlGuiInitHelper() = delete;

  static ErrorCode init(const std::any &cfg, UrControlGui &gui);

private:
  static ErrorCode initLayout(const UrControlGuiLayoutConfig &cfg,
                              UrControlGui &gui);

  static ErrorCode initDashboardHelper(UrControlGui &gui);

  static ErrorCode initUrControlGuiExternalBridge(
      const UrContolGuiExternalBridgeConfig &cfg, UrControlGui &gui);
};

#endif /* UR_CONTROL_GUI_URCONTROLGUIINITHELPER_H_ */
