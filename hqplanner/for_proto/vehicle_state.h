#ifndef HQPLANNER_CLASSFORPROTO_VEHICLESTATE_H_
#define HQPLANNER_CLASSFORPROTO_VEHICLESTATE_H_
#include <geometry_msgs/Pose2D.h>

class VehicleSstate {
 public:
  VehicleSstate(geometry_msgs::Pose2D pose, geometry_msgs::Pose2D velocity,
                geometry_msgs::Pose2D acceleration)
      : pose_(pose), velocity_(velocity), acceleration_(acceleration) {}
  ~VehicleSstate();
  void SetPose(geometry_msgs::Pose2D pose) { pose_ = pose; }
  void SetVelocity(geometry_msgs::Pose2D velocity) { velocity_ = velocity; }
  void SetAcceleration(geometry_msgs::Pose2D acceleration) {
    acceleration_ = acceleration;
  }

  geometry_msgs::Pose2D GetPose() { return pose_; }
  geometry_msgs::Pose2D GetVelocity() { return velocity_; }
  geometry_msgs::Pose2D GetAcceleration() { return acceleration_; }

 private:
  geometry_msgs::Pose2D pose_;
  geometry_msgs::Pose2D velocity_;
  geometry_msgs::Pose2D acceleration_;
};

#endif