#ifndef ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_
#define ROBO_MINER_GUI_MINERCONTROLLEREXTERNALBRIDGE_H_

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_interfaces/srv/field_map_validate.hpp"
#include "robo_miner_interfaces/srv/longest_sequence_validate.hpp"
#include "robo_miner_interfaces/srv/activate_mining_validate.hpp"
#include "game_engine/defines/ActionEventDefines.h"
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

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
  CrystalMinedCb crystalMinedCb;
  SystemShutdownCb systemShutdownCb;
  MovementWatcher *movementWatcher = nullptr;
  SolutionValidator *solutionValidator = nullptr;
};

class MinerControllerExternalBridge: public rclcpp::Node {
public:
  MinerControllerExternalBridge();

  int32_t init(const MinerControllerExternalBridgeOutInterface &interface);

  void publishShutdownController();

  void publishFieldMapRevealed();

private:
  typedef std_msgs::msg::Empty Empty;
  typedef robo_miner_interfaces::msg::FieldPoint FieldPoint;
  typedef robo_miner_interfaces::srv::RobotMove RobotMove;
  typedef robo_miner_interfaces::srv::FieldMapValidate FieldMapValidate;
  typedef robo_miner_interfaces::srv::LongestSequenceValidate LongestSequenceValidate;
  typedef robo_miner_interfaces::srv::ActivateMiningValidate ActivateMiningValidate;

  int32_t initOutInterface(
      const MinerControllerExternalBridgeOutInterface &outInterface);
  int32_t initCommunication();

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
