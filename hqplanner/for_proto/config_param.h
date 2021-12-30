#ifndef HQPLANNER_FOR_PROTO_CONFIG_PARAM_H_
#define HQPLANNER_FOR_PROTO_CONFIG_PARAM_H_

#include <string>

namespace hqplanner {
namespace forproto {
struct ConfigParam {
  /* data */
  static const double FLAGS_st_max_s;
  static const double FLAGS_trajectory_time_min_interval;  // second
  static const double FLAGS_trajectory_time_max_interval;
  static const double FLAGS_trajectory_time_high_density_period;
  static const std::size_t FLAGS_max_history_frame_num;
  static const double FLAGS_lane_left_width;
  static const double FLAGS_lane_right_width;
  static const double FLAGS_virtual_stop_wall_length;
  static const double FLAGS_virtual_stop_wall_height;
  static const double FLAGS_max_collision_distance;
  static const double FLAGS_look_forward_time_sec;
  static const double FLAGS_look_forward_short_distance;
  static const double FLAGS_look_forward_long_distance;
  static const double FLAGS_look_backward_distance;
  static const double FLAGS_reference_line_sample_step;
  static const int FLAGS_num_reference_points_near_destination;
};

const double ConfigParam::FLAGS_st_max_s = 40;
const double ConfigParam::FLAGS_trajectory_time_min_interval = 0.02;
const double ConfigParam::FLAGS_trajectory_time_max_interval = 0.1;
const double ConfigParam::FLAGS_trajectory_time_high_density_period = 1.0;
const std::size_t ConfigParam::FLAGS_max_history_frame_num = 2;
const double ConfigParam::FLAGS_lane_left_width = 2.0;
const double ConfigParam::FLAGS_lane_right_width = 2.0;
const double ConfigParam::FLAGS_virtual_stop_wall_length = 0.1;
const double ConfigParam::FLAGS_virtual_stop_wall_height = 2.0;
const double ConfigParam::FLAGS_max_collision_distance = 0.1;
const double ConfigParam::FLAGS_look_forward_time_sec = 8.0;
const double ConfigParam::FLAGS_look_forward_short_distance = 150.0;
const double ConfigParam::FLAGS_look_forward_long_distance = 250.0;
const double ConfigParam::FLAGS_look_backward_distance = 30.0;
const double ConfigParam::FLAGS_reference_line_sample_step = 0.1;
const int ConfigParam::FLAGS_num_reference_points_near_destination = 50;
}  // namespace forproto
}  // namespace hqplanner

#endif