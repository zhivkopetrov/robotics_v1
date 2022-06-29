#ifndef ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include "robo_miner_interfaces/srv/query_initial_robot_position.hpp"
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_interfaces/srv/field_map_validate.hpp"
#include "robo_miner_interfaces/srv/longest_sequence_validate.hpp"
#include "robo_miner_interfaces/srv/activate_mining_validate.hpp"
#include "robo_miner_interfaces/msg/user_authenticate.hpp"
#include "game_engine/defines/ActionEventDefines.h"
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_miner_gui/defines/RoboMinerGuiFunctionalDefines.h"

//Forward declarations
class MovementWatcher;
class SolutionValidator;

struct MinerControllerExternalBridgeOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  RobotActCb robotActCb;
  ToggleHelpPageCb toggleHelpPageCb;
  ToggleDebugInfoCb toggleDebugInfoCb;
  SetDebugMsgCb setDebugMsgCb;
  SetUserDataCb setUserDataCb;
  StartAchievementWonAnimCb startAchievementWonAnimCb;
  StartGameLostAnimCb startGameLostAnimCb;
  TileReleavedCb tileReleavedCb;
  RevealFogOfWarTilesCb revealFogOfWarTilesCb;
  CrystalMinedCb crystalMinedCb;
  MovementWatcher *movementWatcher = nullptr;
  SolutionValidator *solutionValidator = nullptr;
};

class MinerControllerExternalBridge: public rclcpp::Node {
public:
  MinerControllerExternalBridge();
  ~MinerControllerExternalBridge() noexcept;

  ErrorCode init(const MinerControllerExternalBridgeOutInterface &interface);

  void publishShutdownController();

  void publishFieldMapRevealed();

private:
  using Empty = std_msgs::msg::Empty;
  using String = std_msgs::msg::String;
  using UserAuthenticate = robo_miner_interfaces::msg::UserAuthenticate;
  using FieldPoint = robo_miner_interfaces::msg::FieldPoint;
  using RobotMove = robo_miner_interfaces::srv::RobotMove;
  using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;
  using FieldMapValidate = robo_miner_interfaces::srv::FieldMapValidate;
  using LongestSequenceValidate = robo_miner_interfaces::srv::LongestSequenceValidate;
  using ActivateMiningValidate = robo_miner_interfaces::srv::ActivateMiningValidate;

  enum class ControllerStatus {
    IDLE, ACTIVE, SHUTTING_DOWN
  };

  ErrorCode initOutInterface(
      const MinerControllerExternalBridgeOutInterface &outInterface);
  ErrorCode initCommunication();

  void handleInitialRobotPosService(
      const std::shared_ptr<QueryInitialRobotPosition::Request> request,
      std::shared_ptr<QueryInitialRobotPosition::Response> response);

  void handleRobotMoveService(const std::shared_ptr<RobotMove::Request> request,
                              std::shared_ptr<RobotMove::Response> response);

  void handleFieldMapValidateService(
      const std::shared_ptr<FieldMapValidate::Request> request,
      std::shared_ptr<FieldMapValidate::Response> response);

  void handleLongestSequenceValidateService(
      const std::shared_ptr<LongestSequenceValidate::Request> request,
      std::shared_ptr<LongestSequenceValidate::Response> response);

  void handleActivateMiningValidateService(
      const std::shared_ptr<ActivateMiningValidate::Request> request,
      std::shared_ptr<ActivateMiningValidate::Response> response);

  void onUserAuthenticateMsg(const UserAuthenticate::SharedPtr msg);
  void onToggleHelpPageMsg(const Empty::SharedPtr msg);
  void onToggleDebugInfoMsg(const Empty::SharedPtr msg);
  void onDebugMsg(const String::SharedPtr msg);

  void handleNormalMove(const FieldPos &robotPos);
  void handleMiningMove(const FieldPos &robotPos);

  void handleMajorError();

  MinerControllerExternalBridgeOutInterface _outInterface;

  rclcpp::Service<QueryInitialRobotPosition>::SharedPtr _initialRobotPosService;
  rclcpp::Service<RobotMove>::SharedPtr _robotMoveService;
  rclcpp::Service<FieldMapValidate>::SharedPtr _fieldMapValidateService;
  rclcpp::Service<LongestSequenceValidate>::SharedPtr _longestSequenceValidateService;
  rclcpp::Service<ActivateMiningValidate>::SharedPtr _activateMiningValidateService;

  rclcpp::Publisher<Empty>::SharedPtr _shutdownControllerPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _fieldMapReveleadedPublisher;

  rclcpp::Subscription<UserAuthenticate>::SharedPtr _userAuthenticateSubscriber;
  rclcpp::Subscription<Empty>::SharedPtr _toggleHelpPageSubscriber;
  rclcpp::Subscription<Empty>::SharedPtr _toggleDebugInfoSubscriber;
  rclcpp::Subscription<String>::SharedPtr _setDebugMsgSubscriber;

  ControllerStatus _controllerStatus = ControllerStatus::IDLE;
};

#endif /* ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_ */
