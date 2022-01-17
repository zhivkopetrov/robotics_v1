//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include <rclcpp/rclcpp.hpp>

#include "game_engine/Application.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/RoboCollectorGui.h"
#include "robo_collector_gui/config/RoboCollectorGuiConfigGenerator.h"

int32_t main(int32_t argc, char *args[]) {
  rclcpp::init(argc, args);

  const auto engineCfg =
      RoboCollectorGuiConfigGenerator::generateEngineConfig();
  const auto gameCfg = RoboCollectorGuiConfigGenerator::generateGameConfig();

  std::unique_ptr<Game> game = std::make_unique<RoboCollectorGui>();
  Application app(std::move(game));
  if (SUCCESS != app.init(engineCfg, gameCfg)) {
    LOGERR("app.init() failed");
    return FAILURE;
  }

  return app.run();
}

//#include <chrono>
//
//#include "rclcpp/rclcpp.hpp"
//#include "robo_collector_interfaces/msg/direction.hpp"
//#include <iostream>
//
//using namespace std::chrono_literals;
//
///* This example creates a subclass of Node and uses a fancy C++11 lambda
// * function to shorten the callback syntax, at the expense of making the
// * code somewhat more difficult to understand at first glance. */
//
//class MinimalPublisher : public rclcpp::Node
//{
//public:
//  MinimalPublisher()
//  : Node("minimal_publisher"), count_(0)
//  {
//    publisher_ = create_publisher<robo_collector_interfaces::msg::Direction>("direction", 10);
//    auto timer_callback =
//      [this]() -> void {
//    	robo_collector_interfaces::msg::Direction msg;
//    	msg.direction = robo_collector_interfaces::msg::Direction::DIRECTION_UP;
//        RCLCPP_INFO(this->get_logger(),
//        		"Publishing dir: '%hhu'", msg.direction);
//        publisher_->publish(msg);
//      };
//    timer_ = this->create_wall_timer(1s, timer_callback);
//  }
//
//private:
//  rclcpp::TimerBase::SharedPtr timer_;
//  rclcpp::Publisher<robo_collector_interfaces::msg::Direction>::SharedPtr publisher_;
//  size_t count_;
//};
//
//#include <iostream>
//
//int main(int argc, char * argv[])
//{
//  rclcpp::init(argc, argv);
//  rclcpp::spin(std::make_shared<MinimalPublisher>());
//  rclcpp::shutdown();
//  return 0;
//}
