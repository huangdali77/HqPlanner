#ifndef HQPLANNER_FORPROTO_DP_POLY_PATH_CONFIG_H_
#define HQPLANNER_FORPROTO_DP_POLY_PATH_CONFIG_H_
namespace hqplanner {
namespace forproto {
struct DpPolyPathConfig {
  const int sample_points_num_each_level = 9;
  const double step_length_max = 15.0;
  const double step_length_min = 8.0;
  const double lateral_sample_offset = 0.5;
  const double lateral_adjust_coeff = 0.5;
  // Trajectory Cost Config
  const double eval_time_interval = 0.1;
  const double path_resolution = 0.1;
  const double obstacle_ignore_distance = 20.0;
  const double obstacle_collision_distance = 0.2;
  const double obstacle_risk_distance = 2.0;
  const double obstacle_collision_cost = 1e3;
  double path_l_cost;
  double path_dl_cost;
  double path_ddl_cost = 14;
  double path_l_cost_param_l0;
  double path_l_cost_param_b;
  double path_l_cost_param_k;
  double path_out_lane_cost;
  double path_end_l_cost;
  double sidepass_distance;
  int navigator_sample_num_each_level;
};

}  // namespace forproto
}  // namespace hqplanner

#endif