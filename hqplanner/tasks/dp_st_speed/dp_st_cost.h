#ifndef HQPLANNER_TASKS_DP_ST_SPEED_DP_ST_COST_H_
#define HQPLANNER_TASKS_DP_ST_SPEED_DP_ST_COST_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// #include "modules/common/proto/pnc_point.pb.h"
#include "hqplanner/for_proto/pnc_point.h"
// #include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "hqplanner/for_proto/dp_st_speed_config.h"
// #include "modules/planning/common/path_obstacle.h"
#include "hqplanner/path_obstacle.h"
// #include "modules/planning/common/speed/st_boundary.h"
#include "hqplanner/speed/st_boundary.h"
// #include "modules/planning/common/speed/st_point.h"
#include "hqplanner/speed/st_point.h"
// #include "modules/planning/tasks/dp_st_speed/st_graph_point.h"
#include "hqplanner/tasks/dp_st_speed/st_graph_point.h"

namespace hqplanner {
namespace tasks {

class DpStCost {
 public:
  explicit DpStCost(
      const hqplanner::forproto::DpStSpeedConfig& dp_st_speed_config,
      const std::vector<const PathObstacle*>& obstacles,
      const hqplanner::forproto::TrajectoryPoint& init_point);

  float GetObstacleCost(const StGraphPoint& point);

  float GetReferenceCost(const STPoint& point,
                         const STPoint& reference_point) const;

  float GetSpeedCost(const STPoint& first, const STPoint& second,
                     const float speed_limit) const;

  float GetAccelCostByTwoPoints(const float pre_speed, const STPoint& first,
                                const STPoint& second);
  float GetAccelCostByThreePoints(const STPoint& first, const STPoint& second,
                                  const STPoint& third);

  float GetJerkCostByTwoPoints(const float pre_speed, const float pre_acc,
                               const STPoint& pre_point,
                               const STPoint& curr_point);
  float GetJerkCostByThreePoints(const float first_speed,
                                 const STPoint& first_point,
                                 const STPoint& second_point,
                                 const STPoint& third_point);

  float GetJerkCostByFourPoints(const STPoint& first, const STPoint& second,
                                const STPoint& third, const STPoint& fourth);

 private:
  float GetAccelCost(const float accel);
  float JerkCost(const float jerk);

  void AddToKeepClearRange(const std::vector<const PathObstacle*>& obstacles);
  static void SortAndMergeRange(
      std::vector<std::pair<float, float>>* keep_clear_range_);
  bool InKeepClearRange(float s) const;

  const hqplanner::forproto::DpStSpeedConfig& config_;
  const std::vector<const PathObstacle*>& obstacles_;
  const hqplanner::forproto::TrajectoryPoint& init_point_;

  float unit_t_ = 0.0;

  std::unordered_map<std::string, int> boundary_map_;
  std::vector<std::vector<std::pair<float, float>>> boundary_cost_;

  std::vector<std::pair<float, float>> keep_clear_range_;

  std::array<float, 200> accel_cost_;
  std::array<float, 400> jerk_cost_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_COST_H_
