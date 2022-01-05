#ifndef HQPLANNER_TASKS_DP_POLY_PATH_OPTIMIZER_H_
#define HQPLANNER_TASKS_DP_POLY_PATH_OPTIMIZER_H_

#include <string>

#include "hqplanner/for_proto/dp_poly_path_config.h"
#include "hqplanner/for_proto/planning_config.h"
#include "hqplanner/tasks/path_optimizer.h"

namespace hqplanner {
namespace tasks {
using hqplanner::forproto::DpPolyPathConfig;

/**
 * @class DpPolyPathOptimizer
 * @brief DpPolyPathOptimizer does path planning with dynamic programming
 * algorithm.
 */
class DpPolyPathOptimizer : public PathOptimizer {
 public:
  DpPolyPathOptimizer();

  bool Init(const PlanningConfig &config) override;

 private:
  bool Process(const SpeedData &speed_data, const ReferenceLine &reference_line,
               const TrajectoryPoint &init_point,
               PathData *const path_data) override;

 private:
  DpPolyPathConfig config_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_DP_POLY_PATH_OPTIMIZER_H_
