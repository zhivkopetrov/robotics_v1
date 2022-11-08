//Corresponding header
#include "urscript_bridge/utils/Tf2Utils.h"

//System headers

//Other libraries headers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Own components headers

void getAngleAxisRepresentation(
    const geometry_msgs::msg::Quaternion &quaternion,
    geometry_msgs::msg::Vector3 &outAngleAxis) {

  tf2::Quaternion linkQuaternionTf;
  tf2::convert(quaternion, linkQuaternionTf);
  tf2::Vector3 linkAngleAxisTf = linkQuaternionTf.getAxis()
      * linkQuaternionTf.getAngle();

  constexpr tf2Scalar epsilon = 0.00001;
  for (auto &value : linkAngleAxisTf.m_floats) {
    if (epsilon > std::fabs(value)) {
      value = 0.0;
    }
  }

  //convert back to geometry_msg format
  outAngleAxis.x = linkAngleAxisTf.m_floats[0]; //Rx
  outAngleAxis.y = linkAngleAxisTf.m_floats[1]; //Ry
  outAngleAxis.z = linkAngleAxisTf.m_floats[2]; //Rz
}
