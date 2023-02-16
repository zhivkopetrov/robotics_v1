#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOM_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOM_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ur_control_common/external_api/UrControlCommonExternalBridge.h"
#include "ur_control_common/external_api/DashboardProvider.h"
#include "ur_control_common/motion/MotionExecutor.h"
#include "urscript_common/urscript/UrScriptBuilder.h"
#include "ros2_game_engine/communicator/Ros2CommunicatorInterface.h"
#include "game_engine/Game.h"
#include "utils/design_pattern/StateMachine.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_bloom/layout/UrControlBloomLayout.h"
#include "ur_control_bloom/helpers/StateFileHandler.h"

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
  void enterInitState();
  void exitInitState();
  void enterIdleState();
  void exitIdleState();
  void enterBloomState();
  void exitBloomState();
  void enterBloomRecoveryState();
  void exitBloomRecoveryState();
  void enterJengaState();
  void exitJengaState();
  void enterJengaRecoveryState();
  void exitJengaRecoveryState();

  UrControlBloomLayout _layout;
  StateMachine _stateMachine;
  MotionExecutor _motionExecutor;
  std::shared_ptr<UrScriptBuilder> _urScriptBuilder;
  std::shared_ptr<StateFileHandler> _stateFileHandler;

  //ROS2 related objects
  std::shared_ptr<DashboardProvider> _dashboardProvider;
  std::shared_ptr<UrControlCommonExternalBridge> _externalBridge;
  Ros2CommunicatorInterface _communicatorInterface;
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOM_H_ */
