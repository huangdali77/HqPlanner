#ifndef HQPLANNER_FOR_PROTO_DECISION_H_
#define HQPLANNER_FOR_PROTO_DECISION_H_
#include "hqplanner/for_proto/geometry.h"
namespace hqplanner {
namespace forproto {
struct ObjectSidePass {
  enum Type { LEFT = 1, RIGHT = 2 };
  Type type;
};

struct ObjectNudge {
  enum Type {
    LEFT_NUDGE = 1,   // drive from the left side of the obstacle
    RIGHT_NUDGE = 2,  // drive from the right side of the obstacle
    NO_NUDGE = 3      // No nudge is set.
  };
  Type type = NO_NUDGE;
  // minimum lateral distance in meters. positive if type = LEFT_NUDGE
  // negative if type = RIGHT_NUDGE
  double distance_l = 2;
};

struct ObjectIgnore {};
struct ObjectDecisionType {
  enum ObjectTag {
    IGNORE = 1,
    STOP = 2,
    FOLLOW = 3,
    YIELD = 4,
    OVERTAKE = 5,
    NUDGE = 6,
    SIDEPASS = 7,
    AVOID = 8,
    OBJECT_TAG_NOT_SET = 9
  };

  bool has_ignore() { return object_tag == IGNORE; }
  bool has_sidepass() { return object_tag == SIDEPASS; }
  bool has_nudge() { return object_tag == NUDGE; }

  ObjectTag object_tag = OBJECT_TAG_NOT_SET;
  ObjectIgnore ignore_;
  ObjectNudge nudge_;
  ObjectSidePass sidepass_;
  ObjectIgnore ignore() { return ignore_; }
  ObjectNudge nudge() { return nudge_; }
  ObjectSidePass sidepass() { return sidepass_; }
};

enum StopReasonCode {
  STOP_REASON_HEAD_VEHICLE = 1,
  STOP_REASON_DESTINATION = 2,
  STOP_REASON_PEDESTRIAN = 3,
  STOP_REASON_OBSTACLE = 4,
  STOP_REASON_PREPARKING = 5,
  STOP_REASON_SIGNAL = 100,  // only for red signal
  STOP_REASON_STOP_SIGN = 101,
  STOP_REASON_YIELD_SIGN = 102,
  STOP_REASON_CLEAR_ZONE = 103,
  STOP_REASON_CROSSWALK = 104,
  STOP_REASON_CREEPER = 105,
  STOP_REASON_REFERENCE_END = 106,  // end of the reference_line
  STOP_REASON_YELLOW_SIGNAL = 107,  // yellow signal
  STOP_REASON_PULL_OVER = 108       // pull over
};
struct MainStop {
  StopReasonCode reason_code;
  std::string reason;
  // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading;
  // optional apollo.routing.ChangeLaneType change_lane_type = 5;
};

struct ObjectStop {
  StopReasonCode reason_code;
  double distance_s;  // in meters
  // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading;
  std::vector<std::string> wait_for_obstacle;
}

}  // namespace forproto
}  // namespace hqplanner
