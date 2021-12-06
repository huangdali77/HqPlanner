#ifndef HQPLANNER_PATH_DECISION_H_
#define HQPLANNER_PATH_DECISION_H_

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "obstacle.h"
#include "path_obstacle.h"

namespace hqplanner {

/**
 * @class PathDecision
 *
 * @brief PathDecision represents all obstacle decisions on one path.
 */
class PathDecision {
 public:
  PathDecision() = default;

  void AddPathObstacle(PathObstacle path_obstacle);

  const std::unordered_map<std::string, PathObstacle> &path_obstacles() const;

  PathObstacle Find(const std::string &object_id);

 private:
  std::unordered_map<std::string, PathObstacle> path_obstacles_;
};

// ====================函数实现=============================
void PathDecision::AddPathObstacle(PathObstacle path_obstacle) {
  std::pair<std::string, PathObstacle> path_obstacle_t(path_obstacle.Id(),
                                                       path_obstacle);

  path_obstacles_.insert(path_obstacle_t);
}

const std::unordered_map<std::string, PathObstacle>
    &PathDecision::path_obstacles() const {
  return path_obstacles_;
}

PathObstacle PathDecision::Find(const std::string &object_id) {
  if (path_obstacles_.find(object_id) == path_obstacles_.end()) {
    return PathObstacle();
  }
  return path_obstacles_[object_id];
}

}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_PATH_DECISION_H_
