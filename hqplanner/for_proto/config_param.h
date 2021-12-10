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
};

const double ConfigParam::FLAGS_st_max_s = 40;
const double ConfigParam::FLAGS_trajectory_time_min_interval = 0.02;
const double ConfigParam::FLAGS_trajectory_time_max_interval = 0.1;
const double ConfigParam::FLAGS_trajectory_time_high_density_period = 1.0;
const std::size_t ConfigParam::FLAGS_max_history_frame_num = 2;
const double ConfigParam::FLAGS_lane_left_width = 2.0;
const double ConfigParam::FLAGS_lane_right_width = 2.0;
const double ConfigParam::FLAGS_virtual_stop_wall_length = 0.1;
}  // namespace forproto
}  // namespace hqplanner

#endif