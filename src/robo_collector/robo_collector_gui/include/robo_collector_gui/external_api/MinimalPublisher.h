#ifndef ROBO_COLLECTOR_GUI_MINIMALPUBLISHER_H_
#define ROBO_COLLECTOR_GUI_MINIMALPUBLISHER_H_

//C system headers

//C++ system headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "robo_collector_interfaces/msg/direction.hpp"

//Own components headers

//Forward declarations

class MinimalPublisher: public rclcpp::Node {
public:
  MinimalPublisher();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<robo_collector_interfaces::msg::Direction>::SharedPtr publisher_;
  size_t count_;


//  rclcpp::Subscription<robo_collector_interfaces::msg::Direction>::SharedPtr
//    _playerDirSubscriber;
};

#endif /* ROBO_COLLECTOR_GUI_MINIMALPUBLISHER_H_ */
