#ifndef HQPLANNER_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
#define HQPLANNER_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_

#include <vector>

// #include "modules/planning/proto/planning.pb.h"
#include <algorithm>
#include <limits>
#include <utility>

#include "math/linear_interpolation.h"
#include "math/vec2d.h"
#include "trajectory/trajectory.h"

namespace hqplanner {
namespace trajectory {
using hqplanner::forproto::TrajectoryPoint;

class DiscretizedTrajectory : public Trajectory {
 public:
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */
  //   explicit DiscretizedTrajectory(const ADCTrajectory& trajectory);

  explicit DiscretizedTrajectory(
      const std::vector<TrajectoryPoint>& trajectory_points);

  void SetTrajectoryPoints(
      const std::vector<TrajectoryPoint>& trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  TrajectoryPoint StartPoint() const override;

  double GetTemporalLength() const override;

  double GetSpatialLength() const override;

  TrajectoryPoint Evaluate(const double relative_time) const override;

  virtual uint32_t QueryNearestPoint(const double relative_time) const;

  virtual uint32_t QueryNearestPoint(
      const hqplanner::math::Vec2d& position) const;

  virtual void AppendTrajectoryPoint(const TrajectoryPoint& trajectory_point);

  template <typename Iter>
  void PrependTrajectoryPoints(Iter begin, Iter end) {
    if (!trajectory_points_.empty() && begin != end) {
      CHECK((end - 1)->relative_time() <
            trajectory_points_.front().relative_time());
    }
    trajectory_points_.insert(trajectory_points_.begin(), begin, end);
  }

  const TrajectoryPoint& TrajectoryPointAt(const std::uint32_t index) const;

  uint32_t NumOfPoints() const;

  const std::vector<TrajectoryPoint>& trajectory_points() const;
  std::vector<TrajectoryPoint>& trajectory_points();

  virtual void Clear();

 protected:
  std::vector<TrajectoryPoint> trajectory_points_;
};

// ========================函数实现================================
inline std::uint32_t DiscretizedTrajectory::NumOfPoints() const {
  return trajectory_points_.size();
}

inline const std::vector<TrajectoryPoint>&
DiscretizedTrajectory::trajectory_points() const {
  return trajectory_points_;
}

inline std::vector<TrajectoryPoint>&
DiscretizedTrajectory::trajectory_points() {
  return trajectory_points_;
}

inline void DiscretizedTrajectory::SetTrajectoryPoints(
    const std::vector<TrajectoryPoint>& trajectory_points) {
  trajectory_points_ = trajectory_points;
}

inline void DiscretizedTrajectory::Clear() { trajectory_points_.clear(); }

// ======================================================

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points) {
  assert(!trajectory_points.empty());
  trajectory_points_ = trajectory_points;
}

// DiscretizedTrajectory::DiscretizedTrajectory(const ADCTrajectory& trajectory)
// {
//   trajectory_points_.assign(trajectory.trajectory_point().begin(),
//                             trajectory.trajectory_point().end());
// }

TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const {
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time < relative_time;
  };

  auto it_lower =
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, comp);

  if (it_lower == trajectory_points_.begin()) {
    return trajectory_points_.front();
  } else if (it_lower == trajectory_points_.end()) {
    return trajectory_points_.back();
  }
  return hqplanner::math::InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, relative_time);
}

std::uint32_t DiscretizedTrajectory::QueryNearestPoint(
    const double relative_time) const {
  assert(!trajectory_points_.empty());

  if (relative_time >= trajectory_points_.back().relative_time) {
    return trajectory_points_.size() - 1;
  }
  auto func = [](const TrajectoryPoint& tp, const double relative_time) {
    return tp.relative_time < relative_time;
  };
  auto it_lower =
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, func);
  return std::distance(trajectory_points_.begin(), it_lower);
}

std::uint32_t DiscretizedTrajectory::QueryNearestPoint(
    const hqplanner::math::Vec2d& position) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  std::uint32_t index_min = 0;
  for (std::uint32_t i = 0; i < trajectory_points_.size(); ++i) {
    const hqplanner::math::Vec2d curr_point(trajectory_points_[i].path_point.x,
                                            trajectory_points_[i].path_point.y);

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {
  if (!trajectory_points_.empty()) {
    assert(trajectory_point.relative_time >
           trajectory_points_.back().relative_time);
    // CHECK_GT(trajectory_point.relative_time(),
    //          trajectory_points_.back().relative_time());
  }
  trajectory_points_.push_back(trajectory_point);
}

const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const std::uint32_t index) const {
  assert(index < NumOfPoints());
  //   CHECK_LT(index, NumOfPoints());
  return trajectory_points_[index];
}

TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  assert(!trajectory_points_.empty());
  return trajectory_points_.front();
}

double DiscretizedTrajectory::GetTemporalLength() const {
  assert(!trajectory_points_.empty());
  return trajectory_points_.back().relative_time -
         trajectory_points_.front().relative_time;
}

double DiscretizedTrajectory::GetSpatialLength() const {
  assert(!trajectory_points_.empty());
  return trajectory_points_.back().path_point.s -
         trajectory_points_.front().path_point.s;
}
}  // namespace trajectory
}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
