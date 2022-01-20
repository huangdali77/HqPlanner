#ifndef HQPLANNER_TASKS_POLY_ST_POLY_ST_SPEED_OPTIMIZER_H_
#define HQPLANNER_TASKS_POLY_ST_POLY_ST_SPEED_OPTIMIZER_H_

// #include "modules/common/configs/proto/vehicle_config.pb.h"
#include "hqplanner/for_proto/vehicle_config.h"
// #include "modules/planning/proto/planning_config.pb.h"
#include "hqplanner/for_proto/planning_config.h"
// #include "modules/planning/proto/poly_st_speed_config.pb.h"
#include "hqplanner/for_proto/poly_st_speed_config.h"
// #include "modules/planning/proto/st_boundary_config.pb.h"
#include "hqplanner/for_proto/st_boundary_config.h"
// #include "modules/planning/tasks/speed_optimizer.h"
#include "hqplanner/tasks/speed_optimizer.h"
// #include "modules/planning/tasks/st_graph/st_boundary_mapper.h"
#include "hqplanner/tasks/st_graph/st_boundary_mapper.h"

namespace hqplanner {
namespace tasks {

class PolyStSpeedOptimizer : public SpeedOptimizer {
 public:
  PolyStSpeedOptimizer();

  bool Init(const PlanningConfig& config) override;

 private:
  bool Process(const SLBoundary& adc_sl_boundary, const PathData& path_data,
               const forproto::TrajectoryPoint& init_point,
               const ReferenceLine& reference_line,
               const SpeedData& reference_speed_data,
               PathDecision* const path_decision,
               SpeedData* const speed_data) override;

  hqplanner::forproto::PolyStSpeedConfig poly_st_speed_config_;
  hqplanner::forproto::StBoundaryConfig st_boundary_config_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_POLY_ST_POLY_ST_SPEED_OPTIMIZER_H_
