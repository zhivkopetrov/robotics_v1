#ifndef URSCRIPT_BRIDGE_TF2UTILS_H_
#define URSCRIPT_BRIDGE_TF2UTILS_H_

//System headers
#include <string>

//Other libraries headers
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/vector3.hpp>

//Own components headers

//Forward declarations

/* gets angle-axis representation behind parent frame-id link (if such exists)
 * returns true if such link exists with populated angle-axis representation
 * */
bool getAngleAxisRepresentationForLink(
    const tf2_msgs::msg::TFMessage &state, std::string_view linkName,
    geometry_msgs::msg::Vector3 &outAngleAxis, std::string &outErrorCode);

#endif /* URSCRIPT_BRIDGE_TF2UTILS_H_ */
