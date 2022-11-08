#ifndef URSCRIPT_BRIDGE_TF2UTILS_H_
#define URSCRIPT_BRIDGE_TF2UTILS_H_

//System headers
#include <string>

//Other libraries headers
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

//Own components headers

//Forward declarations

void getAngleAxisRepresentation(
    const geometry_msgs::msg::Quaternion &quaternion,
    geometry_msgs::msg::Vector3 &outAngleAxis);

#endif /* URSCRIPT_BRIDGE_TF2UTILS_H_ */
