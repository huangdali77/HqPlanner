#ifndef MODULES_PLANNING_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_
#define MODULES_PLANNING_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_

#include <vector>

// #include "modules/common/configs/proto/vehicle_config.pb.h"
#include "hqplanner/for_proto/vehicle_config.h"
// #include "modules/planning/proto/dp_poly_path_config.pb.h"
#include "hqplanner/for_proto/dp_poly_path_config.h"

// #include "modules/common/math/box2d.h"
#include "hqplanner/math/box2d.h"
// #include "modules/planning/common/obstacle.h"
#include "hqplanner/obstacle.h"
// #include "modules/planning/common/path_decision.h"
#include "hqplanner/path_decision.h"
// #include "modules/planning/common/speed/speed_data.h"
#include "hqplanner/speed/speed_data.h"
// #include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "hqplanner/math/curve1d/quintic_polynomial_curve1d.h"
// #include "modules/planning/reference_line/reference_line.h"
#include "hqplanner/reference_line.h"
// #include "modules/planning/tasks/dp_poly_path/comparable_cost.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/tasks/dp_poly_path/comparable_cost.h"
namespace hqplanner {
namespace tasks {
using hqplanner::PathObstacle;
using hqplanner::ReferenceLine;
using hqplanner::forproto::DpPolyPathConfig;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::SLPoint;
using hqplanner::forproto::VehicleParam;
using hqplanner::math::Box2d;
using hqplanner::math::QuinticPolynomialCurve1d;
using hqplanner::speed::SpeedData;
class TrajectoryCost {
 public:
  TrajectoryCost() = default;
  explicit TrajectoryCost(const DpPolyPathConfig &config,
                          const ReferenceLine &reference_line,
                          const bool is_change_lane_path,
                          const std::vector<const PathObstacle *> &obstacles,
                          const VehicleParam &vehicle_param,
                          const SpeedData &heuristic_speed_data,
                          const SLPoint &init_sl_point);
  ComparableCost Calculate(const QuinticPolynomialCurve1d &curve,
                           const float start_s, const float end_s,
                           const uint32_t curr_level,
                           const uint32_t total_level);

 private:
  ComparableCost CalculatePathCost(const QuinticPolynomialCurve1d &curve,
                                   const float start_s, const float end_s,
                                   const uint32_t curr_level,
                                   const uint32_t total_level);
  ComparableCost CalculateStaticObstacleCost(
      const QuinticPolynomialCurve1d &curve, const float start_s,
      const float end_s);
  ComparableCost CalculateDynamicObstacleCost(
      const QuinticPolynomialCurve1d &curve, const float start_s,
      const float end_s) const;
  ComparableCost GetCostBetweenObsBoxes(const Box2d &ego_box,
                                        const Box2d &obstacle_box) const;

  //   FRIEND_TEST(AllTrajectoryTests, GetCostFromObsSL);
  ComparableCost GetCostFromObsSL(const float adc_s, const float adc_l,
                                  const SLBoundary &obs_sl_boundary);

  Box2d GetBoxFromSLPoint(const SLPoint &sl, const float dl) const;

  const DpPolyPathConfig config_;
  const ReferenceLine *reference_line_ = nullptr;
  bool is_change_lane_path_ = false;
  const VehicleParam vehicle_param_;
  SpeedData heuristic_speed_data_;
  const SLPoint init_sl_point_;
  uint32_t num_of_time_stamps_ = 0;
  std::vector<std::vector<Box2d>> dynamic_obstacle_boxes_;
  std::vector<float> obstacle_probabilities_;

  std::vector<SLBoundary> static_obstacle_sl_boundaries_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_
