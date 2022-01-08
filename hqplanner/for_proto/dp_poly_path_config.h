#ifndef HQPLANNER_FORPROTO_DP_POLY_PATH_CONFIG_H_
#define HQPLANNER_FORPROTO_DP_POLY_PATH_CONFIG_H_
namespace hqplanner {
namespace forproto {
struct DpPolyPathConfig {
  const int sample_points_num_each_level = 7;
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
  const double path_l_cost = 6.5;
  const double path_dl_cost = 8e3;
  const double path_ddl_cost = 5e1;
  const double path_l_cost_param_l0 = 1.50;
  const double path_l_cost_param_b = 0.40;
  const double path_l_cost_param_k = 1.50;
  const double path_out_lane_cost = 1e8;
  const double path_end_l_cost = 1.0e4;
  const double sidepass_distance = 2.8;
  const int navigator_sample_num_each_level = 3;
};

}  // namespace forproto
}  // namespace hqplanner

#endif