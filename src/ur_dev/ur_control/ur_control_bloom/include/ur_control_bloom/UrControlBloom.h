#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOM_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOM_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ur_control_common/layout/UrControlCommonLayout.h"
#include "ur_control_common/external_api/UrControlCommonExternalBridge.h"
#include "ur_control_common/external_api/DashboardProvider.h"
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"
#include "game_engine/Game.h"
#include "utils/design_pattern/StateMachine.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class InputEvent;

class UrControlBloom final : public Game {
public:
  friend class UrControlBloomInitHelper;

  UrControlBloom(const Ros2CommunicatorInterface &communicatorOutInterface);

  ErrorCode init(const std::any &cfg) override;
  void deinit() override;

  void draw() const override;
  void handleEvent(const InputEvent &e) override;

  void process() override;

private:
  UrControlCommonLayout _layout;
  StateMachine _stateMachine;

  std::shared_ptr<DashboardProvider> _dashboardProvider;
  std::shared_ptr<UrControlCommonExternalBridge> _externalBridge;
  Ros2CommunicatorInterface _communicatorInterface;
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOM_H_ */
