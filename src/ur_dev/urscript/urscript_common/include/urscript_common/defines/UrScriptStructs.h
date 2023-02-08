#ifndef URSCRIPT_COMMON_URSCRIPTSTRUCTS_H_
#define URSCRIPT_COMMON_URSCRIPTSTRUCTS_H_

//System headers
#include <array>

//Other libraries headers

//Own components headers
#include "urscript_common/defines/UrScriptDefines.h"

//Forward declarations

struct Point3d {
  Point3d() = default;
  Point3d(double inputX, double inputY, double inputZ);

  std::string serialize() const;

  double x { };
  double y { };
  double z { };
};

struct AngleAxis {
  AngleAxis() = default;
  AngleAxis(double inputRX, double inputRY, double inputRZ);

  std::string serialize() const;

  double rx { };
  double ry { };
  double rz { };
};

struct WaypointCartesian {
  WaypointCartesian() = default;
  WaypointCartesian(
    const Point3d& inputPos, const AngleAxis& inputOrientation);

  std::string serialize() const;

  Point3d pos;
  AngleAxis orientation;
};

using Ur10eJoints = std::array<double, UR_10E_JOINTS_COUNT>;

struct WaypointJoint {
  WaypointJoint() = default;
  WaypointJoint(const Ur10eJoints& intputJoints);

  std::string serialize() const;

  Ur10eJoints joints { };
};

struct MoveCommandBase {
  MoveCommandBase() = default;
  MoveCommandBase(
    double inputVelocity, double inputAcceleration, double inputBlendingRadius);

  std::string serialize() const;

  double velocity { };
  double acceleration { };
  double blendingRadius { };
};

struct MoveLinearCommand : public MoveCommandBase {
  MoveLinearCommand() = default;
  MoveLinearCommand(
    const WaypointCartesian& inputWaypoint, double inputVelocity,
    double inputAcceleration, double inputBlendingRadius);

  UrScriptPayload serialize() const;

  WaypointCartesian waypoint;
};

struct MoveJointCommand : public MoveCommandBase {
  MoveJointCommand() = default;
  MoveJointCommand(
    const WaypointJoint& inputWaypoint, double inputVelocity,
    double inputAcceleration, double inputBlendingRadius);

  UrScriptPayload serialize() const;

  WaypointJoint waypoint;
};

#endif /* URSCRIPT_COMMON_URSCRIPTSTRUCTS_H_ */