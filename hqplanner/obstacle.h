#ifndef HQPLANNER_OBSTACLE_H_
#define HQPLANNER_OBSTACLE_H_

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "for_proto/config_param.h"
#include "for_proto/perception_obstacle.h"
#include "for_proto/pnc_point.h"
#include "for_proto/prediction_obstacle.h"
#include "math/box2d.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
// #include "modules/planning/common/indexed_list.h"
// #include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace hqplanner {
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::Point;
using hqplanner::forproto::PredictionObstacles;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::math::Box2d;
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
  bool IsVirtual() const;

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
  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const prediction::PredictionObstacles &predictions);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::string &id, const Box2d &obstacle_box);

  static bool IsStaticObstacle(const PerceptionObstacle &perception_obstacle);

  static bool IsVirtualObstacle(const PerceptionObstacle &perception_obstacle);

  //   static bool IsValidTrajectoryPoint(const common::TrajectoryPoint &point);

  const PerceptionObstacle &Perception() const;

 private:
  std::string id_;
  std::int32_t perception_id_ = 0;
  bool is_static_ = true;
  bool is_virtual_ = false;
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

  is_virtual_ = IsVirtualObstacle(perception_obstacle);
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

std::unique_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const std::string &id, const Box2d &obstacle_box) {
  // create a "virtual" perception_obstacle
  PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  int32_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.id = negative_id;
  perception_obstacle.position.x = obstacle_box.center().x();
  perception_obstacle.position.y = obstacle_box.center().y();

  // perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  // perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.theta = obstacle_box.heading();

  perception_obstacle.velocity.x = 0;
  perception_obstacle.velocity.y = 0;

  perception_obstacle.length = obstacle_box.length();
  perception_obstacle.width = obstacle_box.width();
  perception_obstacle.height = ConfigParam::FLAGS_virtual_stop_wall_height;
  perception_obstacle.type = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  perception_obstacle.tracking_time = 1.0;

  std::vector<Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto &corner_point : corner_points) {
    Point point = {corner_point.x(), corner_point.y(), 0};
    perception_obstacle.polygon_point.emplace_back(std::move(point));
    // auto *point = perception_obstacle.add_polygon_point();
    // point->set_x(corner_point.x());
    // point->set_y(corner_point.y());
  }
  auto *obstacle = new Obstacle(id, perception_obstacle);
  obstacle->is_virtual_ = true;
  return std::unique_ptr<Obstacle>(obstacle);
}
bool Obstacle::IsVirtual() const { return is_virtual_; }

bool Obstacle::IsVirtualObstacle(
    const PerceptionObstacle &perception_obstacle) {
  return perception_obstacle.id < 0;
}

std::list<std::unique_ptr<Obstacle>> Obstacle::CreateObstacles(
    const PredictionObstacles &predictions) {
  std::list<std::unique_ptr<Obstacle>> obstacles;

  for (const auto &prediction_obstacle : predictions.prediction_obstacle) {
    const auto perception_id =
        std::to_string(prediction_obstacle.perception_obstacle.id);

    if (prediction_obstacle.trajectory.empty()) {
      obstacles.emplace_back(
          new Obstacle(perception_id, prediction_obstacle.perception_obstacle));
      continue;
    }

    obstacles.emplace_back(
        new Obstacle(perception_id, prediction_obstacle.perception_obstacle,
                     prediction_obstacle.trajectory.front()));
  }
  return obstacles;
}

}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_OBSTACLE_H_
