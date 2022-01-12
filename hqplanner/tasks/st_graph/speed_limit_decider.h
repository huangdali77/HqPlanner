#ifndef MODULES_PLANNING_TASKS_ST_GRAPH_SPEED_LIMIT_DECIDER_H_
#define MODULES_PLANNING_TASKS_ST_GRAPH_SPEED_LIMIT_DECIDER_H_

#include <string>
#include <vector>

// #include "modules/common/configs/proto/vehicle_config.pb.h"
#include "hqplanner/for_proto/vehicle_config.h"
// #include "modules/planning/proto/st_boundary_config.pb.h"
#include "hqplanner/for_proto/st_boundary_config.h"

// #include "modules/common/status/status.h"
// #include "modules/planning/common/obstacle.h"
#include "hqplanner/obstacle.h"
// #include "modules/planning/common/path/path_data.h"
#include "hqplanner/path/path_data.h"
// #include "modules/planning/common/path_obstacle.h"
#include "hqplanner/path_obstacle.h"
// #include "modules/planning/common/speed_limit.h"
#include "hqplanner/speed/speed_limit.h"
// #include "modules/planning/reference_line/reference_line.h"
#include "hqplanner/for_proto/sl_boundary.h"
#include "hqplanner/reference_line.h"

namespace apollo {
namespace planning {
using hqplanner::ReferenceLine;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::StBoundaryConfig;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedLimit;
class SpeedLimitDecider {
 public:
  SpeedLimitDecider(const SLBoundary& adc_sl_boundary,
                    const StBoundaryConfig& config,
                    const ReferenceLine& reference_line,
                    const PathData& path_data);

  virtual ~SpeedLimitDecider() = default;

  virtual bool GetSpeedLimits(
      const IndexedList<std::string, PathObstacle>& path_obstacles,
      SpeedLimit* const speed_limit_data) const;

 private:
  //   FRIEND_TEST(SpeedLimitDeciderTest, get_centric_acc_limit);
  double GetCentricAccLimit(const double kappa) const;

  void GetAvgKappa(
      const std::vector<hqplanner::forproto::PathPoint>& path_points,
      std::vector<double>* kappa) const;

 private:
  const SLBoundary& adc_sl_boundary_;
  const StBoundaryConfig& st_boundary_config_;
  const ReferenceLine& reference_line_;
  const PathData& path_data_;
  const hqplanner::forproto::VehicleParam& vehicle_param_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_SPEED_LIMIT_DECIDER_H_
