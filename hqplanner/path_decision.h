#ifndef HQPLANNER_PATH_DECISION_H_
#define HQPLANNER_PATH_DECISION_H_

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hqplanner/for_proto/decision.h"
#include "hqplanner/for_proto/geometry.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/path_obstacle.h"
#include "hqplanner/reference_line.h"
#include "hqplanner/speed/st_boundary.h"
#include "obstacle.h"
#include "path_obstacle.h"

namespace hqplanner {
using hqplanner::PathObstacle;
using hqplanner::ReferenceLine;
using hqplanner::forproto::MainStop;
using hqplanner::forproto::ObjectDecisionType;
using hqplanner::forproto::ObjectStop;
using hqplanner::forproto::PointENU;
using hqplanner::forproto::SLPoint;
using hqplanner::speed::StBoundary;
/**
 * @class PathDecision
 *
 * @brief PathDecision represents all obstacle decisions on one path.
 */
class PathDecision {
 public:
  PathDecision() = default;

  // void AddPathObstacle(PathObstacle path_obstacle);
  PathObstacle *AddPathObstacle(const PathObstacle &path_obstacle);

  const std::unordered_map<std::string, hqplanner::PathObstacle>
      &path_obstacles() const;

  // PathObstacle Find(const std::string &object_id);
  bool AddLateralDecision(
      const std::string &tag, const std::string &object_id,
      const hqplanner::forproto::ObjectDecisionType &decision);
  bool AddLongitudinalDecision(
      const std::string &tag, const std::string &object_id,
      const hqplanner::forproto::ObjectDecisionType &decision);
  void SetStBoundary(const std::string &id,
                     const hqplanner::speed::StBoundary &boundary);
  void EraseStBoundaries();
  hqplanner::forproto::MainStop main_stop() const { return main_stop_; }
  const PathObstacle *Find(const std::string &object_id) const;

  PathObstacle *Find(const std::string &object_id);

  double stop_reference_line_s() const { return stop_reference_line_s_; }
  bool MergeWithMainStop(const ObjectStop &obj_stop, const std::string &obj_id,
                         const ReferenceLine &ref_line,
                         const SLBoundary &adc_sl_boundary);
  const std::vector<const PathObstacle *> path_obstacle_items() const {
    return path_obstacle_items_;
  }

 private:
  std::unordered_map<std::string, PathObstacle> path_obstacles_;
  std::vector<const PathObstacle *> path_obstacle_items_;
  double stop_reference_line_s_ = std::numeric_limits<double>::max();
  hqplanner::forproto::MainStop main_stop_;
};

// ====================函数实现=============================
// PathObstacle *PathDecision::AddPathObstacle(const PathObstacle
// &path_obstacle) {
//   auto obs = path_obstacles_.find(path_obstacle.Id());

//   if (obs != path_obstacles_.end()) {
//     path_obstacles_.erase(path_obstacle.Id());
//     path_obstacles_.insert(std::make_pair(path_obstacle.Id(),
//     path_obstacle));

//   } else {
//     path_obstacles_.insert(std::make_pair(path_obstacle.Id(),
//     path_obstacle));
//   }
//   return &path_obstacles_.at(path_obstacle.Id());
// }

// const std::unordered_map<std::string, PathObstacle>
//     &PathDecision::path_obstacles() const {
//   return path_obstacles_;
// }

// PathObstacle *PathDecision::Find(const std::string &object_id) {
//   // return path_obstacles_.Find(object_id);
//   if (path_obstacles_.find(object_id) == path_obstacles_.end()) {
//     return nullptr;
//   }
//   return &path_obstacles_[object_id];
// }

// void PathDecision::SetStBoundary(const std::string &id,
//                                  const StBoundary &boundary) {
//   auto obstacle = path_obstacles_.find(id);

//   if (obstacle == path_obstacles_.end()) {
//     // AERROR << "Failed to find obstacle : " << id;
//     return;
//   } else {
//     obstacle->second.SetStBoundary(boundary);
//     // obstacle->SetStBoundary(boundary);
//   }
// }

// bool PathDecision::AddLateralDecision(const std::string &tag,
//                                       const std::string &object_id,
//                                       const ObjectDecisionType &decision) {
//   auto path_obstacle = path_obstacles_.find(object_id);
//   if (path_obstacle == path_obstacles_.end()) {
//     // AERROR << "failed to find obstacle";
//     return false;
//   }
//   path_obstacle->AddLateralDecision(tag, decision);
//   return true;
// }

}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_PATH_DECISION_H_
