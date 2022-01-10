#include "hqplanner/tasks/dp_poly_path/dp_poly_path_optimizer.h"

#include <string>
#include <utility>
#include <vector>

// #include "modules/common/util/file.h"
#include <assert.h>

#include "hqplanner/for_proto/planning_config.h"
// #include "modules/planning/common/planning_gflags.h"
#include "hqplanner/tasks/dp_poly_path/dp_road_graph.h"
// #include "modules/planning/tasks/dp_poly_path/dp_road_graph.h"
namespace hqplanner {
namespace tasks {

DpPolyPathOptimizer::DpPolyPathOptimizer()
    : PathOptimizer("DpPolyPathOptimizer") {}

bool DpPolyPathOptimizer::Init(const PlanningConfig &config) {
  config_ = config.em_planner_config.dp_poly_path_config;
  is_init_ = true;
  return true;
}

bool DpPolyPathOptimizer::Process(const SpeedData &speed_data,
                                  const ReferenceLine &reference_line,
                                  const TrajectoryPoint &init_point,
                                  PathData *const path_data) {
  if (!is_init_) {
    return false;
  }
  assert(path_data != nullptr);
  //   CHECK_NOTNULL(path_data);
  DPRoadGraph dp_road_graph(config_, *reference_line_info_, speed_data);
  // dp_road_graph.SetDebugLogger(reference_line_info_->mutable_debug());

  std::vector<const PathObstacle *> path_obstacles;
  for (auto &path_obstacle :
       reference_line_info_->path_decision()->path_obstacles()) {
    path_obstacles.push_back(&(path_obstacle.second));
  }
  if (!dp_road_graph.FindPathTunnel(init_point, path_obstacles, path_data)) {
    return false;
  }

  return true;
}

}  // namespace tasks
}  // namespace hqplanner
