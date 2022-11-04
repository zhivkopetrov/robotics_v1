//Corresponding header
#include "urscript_bridge/utils/Tf2Utils.h"

//System headers
#include <algorithm>

//Other libraries headers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Own components headers

bool getAngleAxisRepresentationForLink(
    const tf2_msgs::msg::TFMessage &state, std::string_view linkName,
    geometry_msgs::msg::Vector3 &outAngleAxis, std::string &outErrorCode) {

  auto it = std::find_if(state.transforms.begin(), state.transforms.end(),
      [&](const geometry_msgs::msg::TransformStamped &transform) {
        return transform.header.frame_id == linkName;
      });

  if (it == state.transforms.end()) {
    outErrorCode = "Link [";
    outErrorCode.append(linkName).append("] could not be found");
    return false;
  }

  tf2::Quaternion linkQuaternionTf;
  tf2::convert(it->transform.rotation, linkQuaternionTf);
  const tf2::Vector3 linkAngleAxisTf = linkQuaternionTf.getAxis()
      * linkQuaternionTf.getAngle();

  //convert back to geometry_msg format
  outAngleAxis.x = linkAngleAxisTf.m_floats[0]; //Rx
  outAngleAxis.y = linkAngleAxisTf.m_floats[1]; //Ry
  outAngleAxis.z = linkAngleAxisTf.m_floats[2]; //Rz

  return true;
}
