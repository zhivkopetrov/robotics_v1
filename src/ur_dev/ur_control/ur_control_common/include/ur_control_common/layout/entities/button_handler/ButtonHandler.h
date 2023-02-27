#ifndef UR_CONTROL_COMMON_BUTTONHANDLER_H_
#define UR_CONTROL_COMMON_BUTTONHANDLER_H_

//System headers
#include <array>
#include <vector>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/layout/entities/button_handler/config/ButtonHandlerConfig.h"
#include "ur_control_common/layout/entities/button_handler/ButtonHandlerInterfaces.h"
#include "ur_control_common/layout/entities/buttons/DashboardButton.h"

//Forward declarations
class InputEvent;

enum class GripperButtonsInputStatus {
  LOCKED, UNLOCKED
};

class ButtonHandler {
public:
  virtual ~ButtonHandler() noexcept = default;

  virtual ErrorCode init(const ButtonHandlerHighLevelConfig& cfg,
                         const ButtonHandlerOutInterface& outInterface) = 0;
  virtual void draw() const = 0;
  virtual void handleEvent(const InputEvent &e) = 0;
  virtual ErrorCode setCommandButtonsLockStatus(
    const std::vector<int32_t>& lockBtnIndexes,
    const std::vector<int32_t>& unlockBtnIndexes) = 0;
  virtual void setGripperButtonsLockStatus(
    GripperButtonsInputStatus status) = 0;

protected:
  ErrorCode initInternal(
    const ButtonHandlerConfig& cfg,
    const InvokeDashboardServiceCb& invokeDashboardServiceCb);

  ErrorCode sanityCheckCommandButtonsLockStatus(
    const std::vector<int32_t>& lockBtnIndexes,
    const std::vector<int32_t>& unlockBtnIndexes);

  enum DashboardButtonDefines {
    POWER_ON_IDX, 
    POWER_OFF_IDX, 
    BRAKE_RELEASE_IDX,
    DASHBOARD_BUTTONS_COUNT
  };

  std::array<DashboardButton, DASHBOARD_BUTTONS_COUNT> _dashboardButtons;
};

#endif /* UR_CONTROL_COMMON_BUTTONHANDLER_H_ */
