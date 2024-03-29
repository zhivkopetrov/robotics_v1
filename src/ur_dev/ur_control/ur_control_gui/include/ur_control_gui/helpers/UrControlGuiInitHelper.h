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
struct UrControlCommonLayoutConfig;
struct UrContolGuiExternalBridgeConfig;
struct UrControlCommonLayoutInterface;

class UrControlGuiInitHelper {
public:
  UrControlGuiInitHelper() = delete;

  static ErrorCode init(const std::any &cfg, UrControlGui &gui);

private:
  static ErrorCode initLayout(
      const UrControlCommonLayoutConfig &cfg,
      UrControlCommonLayoutInterface &layoutInterface, //out param
      UrControlGui &gui);

  static ErrorCode initDashboardHelper(
      const UrControlCommonLayoutInterface &layoutInterface, UrControlGui &gui);

  static ErrorCode initUrControlGuiExternalBridge(
      const UrContolGuiExternalBridgeConfig &cfg,
      const UrControlCommonLayoutInterface &layoutInterface, UrControlGui &gui);
};

#endif /* UR_CONTROL_GUI_URCONTROLGUIINITHELPER_H_ */
