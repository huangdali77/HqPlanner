// #include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"

#include <cmath>

#include "eigen3/Eigen/Core"

// #include "modules/common/configs/config_gflags.h"
// #include "modules/common/log.h"
// #include "modules/common/math/euler_angles_zxy.h"
// #include "modules/common/math/quaternion.h"
// #include "modules/common/util/string_util.h"
// #include "modules/localization/common/localization_gflags.h"

namespace hqplanner {
namespace forproto {

VehicleStateProvider::VehicleStateProvider() {}

void VehicleStateProvider::UpdateNextCycleVehicleState(
    const VehicleState &vehicle_state) {
  vehicle_state_ = vehicle_state;
}

double VehicleStateProvider::x() const { return vehicle_state_.x; }

double VehicleStateProvider::y() const { return vehicle_state_.y; }

double VehicleStateProvider::z() const { return vehicle_state_.z; }

double VehicleStateProvider::roll() const { return vehicle_state_.roll; }

double VehicleStateProvider::pitch() const { return vehicle_state_.pitch; }

double VehicleStateProvider::yaw() const { return vehicle_state_.yaw; }

double VehicleStateProvider::heading() const { return vehicle_state_.heading; }

double VehicleStateProvider::kappa() const { return vehicle_state_.kappa; }

double VehicleStateProvider::linear_velocity() const {
  return vehicle_state_.linear_velocity;
}

double VehicleStateProvider::angular_velocity() const {
  return vehicle_state_.angular_velocity;
}

double VehicleStateProvider::linear_acceleration() const {
  return vehicle_state_.linear_acceleration;
}

// double VehicleStateProvider::gear() const { return vehicle_state_.gear(); }

double VehicleStateProvider::timestamp() const {
  return vehicle_state_.timestamp;
}

// const localization::Pose &VehicleStateProvider::pose() const {
//   return vehicle_state_.pose();
// }

// const localization::Pose &VehicleStateProvider::original_pose() const {
//   return original_localization_.pose();
// }

void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  vehicle_state_.linear_velocity = linear_velocity;
}

const VehicleState &VehicleStateProvider::vehicle_state() const {
  return vehicle_state_;
}

void VehicleStateProvider::set_vehicle_config(const double x, const double y,
                                              const double heading) {
  vehicle_state_.x = x;
  vehicle_state_.y = y;
  vehicle_state_.heading = heading;
}

math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  double v = vehicle_state_.linear_velocity;

  // Predict distance travel vector
  if (std::fabs(vehicle_state_.angular_velocity) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / vehicle_state_.angular_velocity *
                      (1.0 - std::cos(vehicle_state_.angular_velocity * t));
    vec_distance[1] = std::sin(vehicle_state_.angular_velocity * t) * v /
                      vehicle_state_.angular_velocity;
  }

  // If we have rotation information, take it into consideration.
  //   if (vehicle_state_.pose().has_orientation()) {
  //     const auto &orientation = vehicle_state_.pose().orientation();
  //     Eigen::Quaternion<double> quaternion(orientation.qw(),
  //     orientation.qx(),
  //                                          orientation.qy(),
  //                                          orientation.qz());
  //     Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
  //                             vehicle_state_.z());
  //     auto future_pos_3d = quaternion.toRotationMatrix() * vec_distance +
  //     pos_vec; return math::Vec2d(future_pos_3d[0], future_pos_3d[1]);
  //   }

  // If no valid rotation information provided from localization,
  // return the estimated future position without rotation.
  return math::Vec2d(vec_distance[0] + vehicle_state_.x,
                     vec_distance[1] + vehicle_state_.y);
}

}  // namespace forproto
}  // namespace hqplanner
