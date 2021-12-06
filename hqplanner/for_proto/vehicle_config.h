#ifndef HQPLANNER_FORPROTO_VEHICLE_CONFIG_H_
#define HQPLANNER_FORPROTO_VEHICLE_CONFIG_H_
namespace hqplanner {
namespace forproto {
struct VehicleParam {
  // Car center point is car reference point, i.e., center of rear axle.
  double front_edge_to_center = 2;
  double back_edge_to_center = 3;
  double left_edge_to_center = 4;
  double right_edge_to_center = 5;

  double length = 6;
  double width = 7;
  double height = 8;

  double min_turn_radius = 9;
  double max_acceleration = 10;
  double max_deceleration = 11;

  // The following items are used to compute trajectory constraints in
  // planning/control/canbus, vehicle max steer angle
  double max_steer_angle = 12;
  // vehicle max steer rate; how fast can the steering wheel turn.
  double max_steer_angle_rate = 13;
  // vehicle min steer rate;
  double min_steer_angle_rate = 14;
  // ratio between the turn of steering wheel and the turn of wheels
  double steer_ratio = 15;
  // the distance between the front and back wheels
  double wheel_base = 16;
  // Tire effective rolling radius (vertical distance between the wheel center
  // and the ground).
  double wheel_rolling_radius = 17;

  // minimum differentiable vehicle speed, in m/s
  float max_abs_speed_when_stopped = 18;
};

struct VehicleConfig {
  VehicleParam vehicle_param;
}

}  // namespace forproto
}  // namespace hqplanner

#endif