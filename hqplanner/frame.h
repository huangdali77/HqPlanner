#ifndef HQPLANNER_FRAME_H_
#define HQPLANNER_FRAME_H_
// #include <fsd_common_msgs/CarState.h>
// #include <fsd_common_msgs/CarStateDt.h>
// #include <fsd_common_msgs/TrajectoryPoint.h>

#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "for_proto/pnc_point.h"
#include "for_proto/vehicle_state.h"
#include "reference_line.h"
namespace hqplanner {
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleState;

class Frame {
 public:
  Frame() = default;
  explicit Frame(uint32_t sequence_num,
                 const TrajectoryPoint &planning_start_point,
                 const double start_time, const VehicleState &vehicle_state,
                 ReferenceLineProvider *reference_line_provider);

 private:
  VehicleState vehicle_state_;
  uint32_t sequence_num_;
  TrajectoryPoint planning_start_point_;
  double start_time_;
  std::list<ReferenceLine>
};
}  // namespace hqplanner

#endif