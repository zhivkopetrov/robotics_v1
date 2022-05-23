#ifndef ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_interfaces/srv/field_map_validate.hpp"
#include "robo_miner_interfaces/srv/longest_sequence_validate.hpp"
#include "robo_miner_interfaces/srv/activate_mining_validate.hpp"
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
  StartAchievementWonAnimCb startAchievementWonAnimCb;
  StartGameLostAnimCb startGameLostAnimCb;
  TileReleavedCb tileReleavedCb;
  RevealFogOfWarTilesCb revealFogOfWarTilesCb;
  CrystalMinedCb crystalMinedCb;
  SystemShutdownCb systemShutdownCb;
  MovementWatcher *movementWatcher = nullptr;
  SolutionValidator *solutionValidator = nullptr;
};

class MinerControllerExternalBridge: public rclcpp::Node {
public:
  MinerControllerExternalBridge();

  ErrorCode init(const MinerControllerExternalBridgeOutInterface &interface);

  void publishShutdownController();

  void publishFieldMapRevealed();

private:
  using Empty = std_msgs::msg::Empty;
  using FieldPoint = robo_miner_interfaces::msg::FieldPoint;
  using RobotMove = robo_miner_interfaces::srv::RobotMove;
  using FieldMapValidate = robo_miner_interfaces::srv::FieldMapValidate;
  using LongestSequenceValidate = robo_miner_interfaces::srv::LongestSequenceValidate;
  using ActivateMiningValidate = robo_miner_interfaces::srv::ActivateMiningValidate;

  ErrorCode initOutInterface(
      const MinerControllerExternalBridgeOutInterface &outInterface);
  ErrorCode initCommunication();

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

  void handleNormalMove(const FieldPos& robotPos);
  void handleMiningMove(const FieldPos& robotPos);

  MinerControllerExternalBridgeOutInterface _outInterface;

  rclcpp::Service<RobotMove>::SharedPtr _robotMoveService;
  rclcpp::Service<FieldMapValidate>::SharedPtr _fieldMapValidateService;
  rclcpp::Service<LongestSequenceValidate>::SharedPtr _longestSequenceValidateService;
  rclcpp::Service<ActivateMiningValidate>::SharedPtr _activateMiningValidateService;

  rclcpp::Publisher<Empty>::SharedPtr _shutdownControllerPublisher;
  rclcpp::Publisher<Empty>::SharedPtr _fieldMapReveleadedPublisher;
};

#endif /* ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_ */
