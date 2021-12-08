#ifndef HQPLANNER_OBSTACLE_H_
#define HQPLANNER_OBSTACLE_H_

#include <assert.h>

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "for_proto/perception_obstacle.h"
#include "for_proto/pnc_point.h"
#include "math/box2d.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
// #include "modules/planning/common/indexed_list.h"
// #include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace hqplanner {
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::math::Polygon2d;
using hqplanner::math::Vec2d;
/**
 * @class Obstacle
 *
 * @brief Obstacle represents one perception obstacle.
 */
class Obstacle {
 public:
  Obstacle() = default;

  Obstacle(const std::string &id,
           const PerceptionObstacle &perception_obstacle);

  //   Obstacle(const std::string &id,
  //            const perception::PerceptionObstacle &perception,
  //            const prediction::Trajectory &trajectory);

  const std::string &Id() const;
  void SetId(const std::string &id) { id_ = id; }

  std::int32_t PerceptionId() const;

  double Speed() const;

  bool IsStatic() const;
  //   bool IsVirtual() const;

  //   TrajectoryPoint GetPointAtTime(const double time) const;

  math::Box2d GetBoundingBox() const;
  /**
   * @brief get the perception bounding box
   */
  const math::Box2d &PerceptionBoundingBox() const;

  /**
   * @brief get the perception polygon for the obstacle. It is more precise than
   * bounding box
   */
  const math::Polygon2d &PerceptionPolygon() const;

  const std::vector<TrajectoryPoint> &Trajectory() const;
  //   TrajectoryPoint *AddTrajectoryPoint();
  //   bool HasTrajectory() const;

  //   const PerceptionObstacle &Perception() const;

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  //   static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
  //       const prediction::PredictionObstacles &predictions);

  //   static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
  //       const std::string &id, const common::math::Box2d &obstacle_box);

  static bool IsStaticObstacle(const PerceptionObstacle &perception_obstacle);

  //   static bool IsVirtualObstacle(
  //       const perception::PerceptionObstacle &perception_obstacle);

  //   static bool IsValidTrajectoryPoint(const common::TrajectoryPoint &point);

  const PerceptionObstacle &Perception() const;

 private:
  std::string id_;
  std::int32_t perception_id_ = 0;
  bool is_static_ = true;
  //   bool is_virtual_ = false;
  double speed_ = 0.0;
  std::vector<TrajectoryPoint> trajectory_;
  PerceptionObstacle perception_obstacle_;
  math::Box2d perception_bounding_box_;
  math::Polygon2d perception_polygon_;
};

// typedef IndexedList<std::string, Obstacle> IndexedObstacles;
// typedef ThreadSafeIndexedList<std::string, Obstacle>
// ThreadSafeIndexedObstacles;

// ==============================函数实现============================
Obstacle::Obstacle(const std::string &id,
                   const PerceptionObstacle &perception_obstacle)
    : id_(id),
      perception_id_(perception_obstacle.id),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_(
          {perception_obstacle_.position.x, perception_obstacle_.position.y},
          perception_obstacle_.theta, perception_obstacle_.length,
          perception_obstacle_.width) {
  std::vector<Vec2d> polygon_points;

  for (const auto &point : perception_obstacle.polygon_point) {
    polygon_points.emplace_back(point.x, point.y);
  }

  assert(Polygon2d::ComputeConvexHull(polygon_points, &perception_polygon_));

  is_static_ = IsStaticObstacle(perception_obstacle);

  speed_ = std::hypot(perception_obstacle.velocity.x,
                      perception_obstacle.velocity.y);
}

bool Obstacle::IsStaticObstacle(const PerceptionObstacle &perception_obstacle) {
  if (perception_obstacle.type == PerceptionObstacle::UNKNOWN_UNMOVABLE) {
    return true;
  }
  auto moving_speed = std::hypot(perception_obstacle.velocity.x,
                                 perception_obstacle.velocity.y);
  return moving_speed <= 0.5;
}

const PerceptionObstacle &Obstacle::Perception() const {
  return perception_obstacle_;
}
const std::string &Obstacle::Id() const { return id_; }
double Obstacle::Speed() const { return speed_; }

bool Obstacle::IsStatic() const { return is_static_; }

const math::Polygon2d &Obstacle::PerceptionPolygon() const {
  return perception_polygon_;
}

const std::vector<TrajectoryPoint> &Obstacle::Trajectory() const {
  return trajectory_;
}
const math::Box2d &Obstacle::PerceptionBoundingBox() const {
  return perception_bounding_box_;
}
math::Box2d Obstacle::GetBoundingBox() const {
  return perception_bounding_box_;
}

}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_OBSTACLE_H_
