//Corresponding header
#include "robo_collector_gui/external_api/MinimalPublisher.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

using namespace std::literals;

//RoboCollectorGui::RoboCollectorGui() : Node("RoboCollectorGui") {
//  _playerDirSubscriber = create_subscription<robo_collector_interfaces::msg::Direction>(
//    "direction",
//    10,
//    [this](robo_collector_interfaces::msg::Direction::UniquePtr msg) {
//      if (robo_collector_interfaces::msg::Direction::DIRECTION_UP == msg->direction) {
//        _robots[Defines::PLAYER_ROBOT_IDX].act(MoveType::FORWARD);
//      } else if (robo_collector_interfaces::msg::Direction::DIRECTION_LEFT == msg->direction) {
//        _robots[Defines::PLAYER_ROBOT_IDX].act(MoveType::ROTATE_LEFT);
//      } else if (robo_collector_interfaces::msg::Direction::DIRECTION_RIGHT == msg->direction) {
//        _robots[Defines::PLAYER_ROBOT_IDX].act(MoveType::ROTATE_RIGHT);
//      }
//    });
//}

MinimalPublisher::MinimalPublisher()
    : Node("minimal_publisher"), count_(0) {
  publisher_ = create_publisher<robo_collector_interfaces::msg::Direction>(
      "direction", 10);
  auto timer_callback = [this]() -> void {
    robo_collector_interfaces::msg::Direction msg;
    msg.direction = robo_collector_interfaces::msg::Direction::DIRECTION_UP;
    RCLCPP_INFO(this->get_logger(), "Publishing dir: '%hhu', count: %zu",
        msg.direction, count_++);
    publisher_->publish(msg);
  };
  timer_ = this->create_wall_timer(1s, timer_callback);
}
