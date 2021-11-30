#ifndef HQPLANNER_FRAME_H_
#define HQPLANNER_FRAME_H_
#include <fsd_common_msgs/CarState.h>
#include <fsd_common_msgs/CarStateDt.h>
#include <fsd_common_msgs/TrajectoryPoint.h>

#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/vehicle_state.h"
class Frame {
 public:
  explicit Frame(uint32_t sequence_num,
                 const TrajectoryPoint &planning_start_point,
                 const double start_time, const VehicleSstate &vehicle_state,
                 ReferenceLineProvider *reference_line_provider);
};

#endif