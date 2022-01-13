#include "hqplanner/path_decision.h"

#include <memory>
#include <utility>
namespace hqpalnner {
using hqplanner::PathObstacle;

PathObstacle *PathDecision::AddPathObstacle(const PathObstacle &path_obstacle) {
  auto obs = path_obstacles_.find(path_obstacle.Id());

  if (obs != path_obstacles_.end()) {
    path_obstacles_.erase(path_obstacle.Id());
    path_obstacles_.insert(std::make_pair(path_obstacle.Id(), path_obstacle));

  } else {
    path_obstacles_.insert(std::make_pair(path_obstacle.Id(), path_obstacle));
  }
  return &path_obstacles_.at(path_obstacle.Id());
}

const std::unordered_map<std::string, PathObstacle>
    &PathDecision::path_obstacles() const {
  return path_obstacles_;
}

PathObstacle *PathDecision::Find(const std::string &object_id) {
  // return path_obstacles_.Find(object_id);
  if (path_obstacles_.find(object_id) == path_obstacles_.end()) {
    return nullptr;
  }
  return &path_obstacles_[object_id];
}

void PathDecision::SetStBoundary(const std::string &id,
                                 const StBoundary &boundary) {
  auto obstacle = path_obstacles_.find(id);

  if (obstacle == path_obstacles_.end()) {
    // AERROR << "Failed to find obstacle : " << id;
    return;
  } else {
    obstacle->second.SetStBoundary(boundary);
    // obstacle->SetStBoundary(boundary);
  }
}

bool PathDecision::AddLateralDecision(const std::string &tag,
                                      const std::string &object_id,
                                      const ObjectDecisionType &decision) {
  auto path_obstacle = path_obstacles_.find(object_id);
  if (path_obstacle == path_obstacles_.end()) {
    // AERROR << "failed to find obstacle";
    return false;
  }
  path_obstacle->AddLateralDecision(tag, decision);
  return true;
}
}  // namespace hqpalnner