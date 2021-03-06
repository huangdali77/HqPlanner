#ifndef HQPLANNER_TASKS_DP_ST_SPEED_DP_ST_SPEED_OPTIMIZER_H_
#define HQPLANNER_TASKS_DP_ST_SPEED_DP_ST_SPEED_OPTIMIZER_H_

#include <string>

#include "hqplanner/for_proto/dp_st_speed_config.h"
// #include "modules/planning/proto/planning_internal.pb.h"
// #include "modules/planning/proto/st_boundary_config.pb.h"
#include "hqplanner/for_proto/st_boundary_config.h"

// #include "modules/planning/tasks/speed_optimizer.h"
#include "hqplanner/tasks/speed_optimizer.h"
// #include "modules/planning/tasks/st_graph/speed_limit_decider.h"
#include "hqplanner/tasks/dp_st_speed/dp_st_speed_optimizer.h"
#include "hqplanner/tasks/st_graph/speed_limit_decider.h"
// #include "modules/planning/tasks/st_graph/st_boundary_mapper.h"
#include "hqplanner/tasks/st_graph/st_boundary_mapper.h"

namespace hqplanner {
namespace tasks {

using hqplanner::forproto::DpStSpeedConfig;
/**
 * @class DpStSpeedOptimizer
 * @brief DpStSpeedOptimizer does ST graph speed planning with dynamic
 * programming algorithm.
 */
class DpStSpeedOptimizer : public SpeedOptimizer {
 public:
  DpStSpeedOptimizer();

  bool Init(const PlanningConfig& config) override;

 private:
  bool Process(const SLBoundary& adc_sl_boundary, const PathData& path_data,
               const TrajectoryPoint& init_point,
               const ReferenceLine& reference_line,
               const SpeedData& reference_speed_data,
               PathDecision* const path_decision,
               SpeedData* const speed_data) override;

  bool SearchStGraph(const StBoundaryMapper& boundary_mapper,
                     const SpeedLimitDecider& speed_limit_decider,
                     const PathData& path_data, SpeedData* speed_data,
                     PathDecision* path_decision) const;

 private:
  TrajectoryPoint init_point_;
  const ReferenceLine* reference_line_ = nullptr;
  SLBoundary adc_sl_boundary_;
  DpStSpeedConfig dp_st_speed_config_;
  StBoundaryConfig st_boundary_config_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_DP_ST_SPEED_OPTIMIZER_H_
