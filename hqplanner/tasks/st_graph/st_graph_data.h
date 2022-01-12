#ifndef HQPLANNER_TASKS_ST_GRAPH_ST_GRAPH_DATA_H_
#define HQPLANNER_TASKS_ST_GRAPH_ST_GRAPH_DATA_H_

#include <vector>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/speed/speed_limit.h"
#include "hqplanner/speed/st_boundary.h"

namespace hqplanner {
namespace tasks {
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::speed::SpeedLimit;
using hqplanner::speed::StBoundary;
class StGraphData {
 public:
  StGraphData(const std::vector<const StBoundary*>& st_boundaries,
              const TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
              const double path_data_length);
  StGraphData() = default;

  const std::vector<const StBoundary*>& st_boundaries() const;

  const TrajectoryPoint& init_point() const;

  const SpeedLimit& speed_limit() const;

  double path_data_length() const;

 private:
  std::vector<const StBoundary*> st_boundaries_;
  TrajectoryPoint init_point_;

  SpeedLimit speed_limit_;
  double path_data_length_ = 0.0;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_DATA_H_
