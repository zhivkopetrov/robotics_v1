//Corresponding header
#include "urscript_common/motion/MotionStructs.h"

//System headers
#include <sstream>

//Other libraries headers

//Own components headers

namespace {
constexpr auto FIXED_PRECISION = 3;
}

Point3d::Point3d(double inputX, double inputY, double inputZ) 
  : x(inputX), y(inputY), z(inputZ) {

}

std::string Point3d::serialize() const {
  std::ostringstream ostr;
  ostr.precision(FIXED_PRECISION);
  ostr << std::fixed << x << "," << y << "," << z;
  return ostr.str();
}

AngleAxis::AngleAxis(double inputRX, double inputRY, double inputRZ) 
  : rx(inputRX), ry(inputRY), rz(inputRZ) {

}

std::string AngleAxis::serialize() const {
  std::ostringstream ostr;
  ostr.precision(FIXED_PRECISION);
  ostr << std::fixed << rx << "," << ry << "," << rz;
  return ostr.str();
}

WaypointCartesian::WaypointCartesian(
    const Point3d& inputPos, const AngleAxis& inputOrientation) 
      : pos(inputPos), orientation(inputOrientation) {

}

std::string WaypointCartesian::serialize() const {
  std::ostringstream ostr;
  ostr << "p[" << pos.serialize() << "," << orientation.serialize() << "]";
  return ostr.str();
}

WaypointJoint::WaypointJoint(const Ur10eJoints& intputJoints) 
  : joints(intputJoints) {

}

std::string WaypointJoint::serialize() const {
  std::ostringstream ostr;
  ostr.precision(FIXED_PRECISION);
  ostr << "[";
  for (int32_t i = BASE_JOINT_IDX; i < WRIST_3_JOINT_IDX; ++i) {
    ostr << std::fixed << joints[i] << ",";
  }
  ostr << std::fixed << joints[WRIST_3_JOINT_IDX] << "]";
  return ostr.str();
}

MoveCommandBase::MoveCommandBase(
    double inputVelocity, double inputAcceleration, double inputBlendingRadius)
      : velocity(inputVelocity), 
      acceleration(inputAcceleration), blendingRadius(inputBlendingRadius) {

}

std::string MoveCommandBase::serialize() const {
  std::ostringstream ostr;
  ostr.precision(FIXED_PRECISION);
  ostr << std::fixed << "a=" << acceleration << ","
       << "v=" << velocity << "," << "t=0.0,r=" << blendingRadius;
  return ostr.str();
}

MoveLinearCommand::MoveLinearCommand(
  const WaypointCartesian& inputWaypoint, double inputVelocity,
  double inputAcceleration, double inputBlendingRadius) 
    : MoveCommandBase(inputVelocity, inputAcceleration, inputBlendingRadius), 
      waypoint(inputWaypoint) {

}

UrScriptPayload MoveLinearCommand::serialize() const {
  std::ostringstream ostr;
  ostr << "movel(" << waypoint.serialize() << "," 
       << MoveCommandBase::serialize() << ")";
  return ostr.str();
}

MoveJointCommand::MoveJointCommand(
  const WaypointJoint& inputWaypoint, double inputVelocity,
  double inputAcceleration, double inputBlendingRadius) 
    : MoveCommandBase(inputVelocity, inputAcceleration, inputBlendingRadius), 
      waypoint(inputWaypoint) {

}

UrScriptPayload MoveJointCommand::serialize() const {
  std::ostringstream ostr;
  ostr << "movej(" << waypoint.serialize() << "," 
       << MoveCommandBase::serialize() << ")";
  return ostr.str();
}