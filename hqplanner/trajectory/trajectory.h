#ifndef HQPLANNER_TRAJECTORY_TRAJECTORY_H_
#define HQPLANNER_TRAJECTORY_TRAJECTORY_H_

#include "for_proto/pnc_point.h"

namespace hqplanner {
namespace trajectory {

using hqplanner::forproto::TrajectoryPoint;
class Trajectory {
 public:
  Trajectory() = default;

  virtual ~Trajectory() = default;

  virtual TrajectoryPoint Evaluate(const double relative_time) const = 0;

  virtual TrajectoryPoint StartPoint() const = 0;

  virtual double GetTemporalLength() const = 0;

  virtual double GetSpatialLength() const = 0;
};

}  // namespace trajectory
}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_H_
