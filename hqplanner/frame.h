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

#include "for_proto/adc_trajectory.h"
#include "for_proto/config_param.h"
#include "for_proto/perception_obstacle.h"
#include "for_proto/pnc_point.h"
#include "for_proto/prediction_obstacle.h"
#include "for_proto/vehicle_state.h"
#include "math/box2d.h"
#include "math/indexed_queue.h"
#include "math/vec2d.h"
#include "obstacle.h"
#include "reference_line.h"
#include "reference_line_info.h"
#include "reference_line_provider.h"
namespace hqplanner {
using hqplanner::forproto::ADCTrajectory;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::PerceptionObstacles;
using hqplanner::forproto::PredictionObstacles;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleState;
using hqplanner::math::Box2d;
using hqplanner::math::IndexedQueue;
using hqplanner::math::Vec2d;

class Frame {
 public:
  Frame() = default;
  explicit Frame(uint32_t sequence_num,
                 const TrajectoryPoint &planning_start_point,
                 const double start_time, const VehicleState &vehicle_state,
                 ReferenceLineProvider *reference_line_provider);

  const TrajectoryPoint &PlanningStartPoint() const;
  bool Init();

  uint32_t SequenceNum() const;

  // std::string DebugString() const;

  // const PublishableTrajectory &ComputedTrajectory() const;

  // void RecordInputDebug(planning_internal::Debug *debug);

  std::list<ReferenceLineInfo> &reference_line_info();

  Obstacle *Find(const std::string &id);

  const ReferenceLineInfo *FindDriveReferenceLineInfo();

  const ReferenceLineInfo *DriveReferenceLineInfo() const;

  const std::vector<const Obstacle *> obstacles() const;

  const Obstacle *CreateStopObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);

  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);

  bool Rerouting();

  const VehicleState &vehicle_state() const;

  static void AlignPredictionTime(const double planning_start_time,
                                  PredictionObstacles *prediction_obstacles);

  ADCTrajectory *mutable_trajectory() { return &trajectory_; }

  const ADCTrajectory &trajectory() const { return trajectory_; }

  const bool is_near_destination() const { return is_near_destination_; }

 private:
  bool CreateReferenceLineInfo();

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such
   * obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle() const;

  /**
   * @brief create a static virtual obstacle
   */
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
                                              const Box2d &box);

  void AddObstacle(const Obstacle &obstacle);

 private:
  uint32_t sequence_num_;
  TrajectoryPoint planning_start_point_;
  double start_time_;
  VehicleState vehicle_state_;
  std::list<ReferenceLineInfo> reference_line_info_;
  bool is_near_destination_ = false;
  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;
  PredictionObstacles prediction_;
  std::unordered_map<std::string, Obstacle> obstacles_;
  ADCTrajectory trajectory_;  // last published trajectory

  ReferenceLineProvider *reference_line_provider_ = nullptr;
};
// =============================函数实现=================================
Frame::Frame(uint32_t sequence_num, const TrajectoryPoint &planning_start_point,
             const double start_time, const VehicleState &vehicle_state,
             ReferenceLineProvider *reference_line_provider)
    : sequence_num_(sequence_num),
      planning_start_point_(planning_start_point),
      start_time_(start_time),
      vehicle_state_(vehicle_state),
      reference_line_provider_(reference_line_provider) {}

const TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

const VehicleState &Frame::vehicle_state() const { return vehicle_state_; }

std::list<ReferenceLineInfo> &Frame::reference_line_info() {
  return reference_line_info_;
}

const Obstacle *Frame::CreateStopObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_s) {
  if (reference_line_info == nullptr) {
    // AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();
  const double box_center_s =
      obstacle_s + ConfigParam::FLAGS_virtual_stop_wall_length / 2.0;
  auto box_center = reference_line.GetReferencePoint(box_center_s);
  double heading = reference_line.GetReferencePoint(obstacle_s).heading;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line.GetLaneWidth(obstacle_s, &lane_left_width, &lane_right_width);
  Box2d stop_wall_box(Vec2d(box_center.x, box_center.y), heading,
                      ConfigParam::FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width);

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

const Obstacle *Frame::CreateStaticVirtualObstacle(const std::string &id,
                                                   const Box2d &box) {
  auto object = obstacles_.find(id);
  if (object != obstacles_.end()) {
    // AWARN << "obstacle " << id << " already exist.";
    return &(object->second);
  }
  auto *ptr = obstacles_.insert(
      std::make_pair(id, *Obstacle::CreateStaticVirtualObstacles(id, box)));
  if (!ptr) {
    AERROR << "Failed to create virtual obstacle " << id;
  }
  return ptr;
}

// ===================FrameHistory======================================
class FrameHistory {
 public:
  // Get infinite capacity with 0.
  FrameHistory() = default;
  //   explicit FrameHistory(std::size_t capacity) : capacity_(capacity) {}

  const Frame *Find(const uint32_t id) const {
    std::unordered_map<uint32_t, std::unique_ptr<Frame>>::const_iterator it =
        map_.find(id);
    if (it == map_.end()) {
      return nullptr;
    }
    return (it->second).get();
  }

  const Frame *Latest() const {
    if (queue_.empty()) {
      return nullptr;
    }
    return Find(queue_.back().first);
  }

  bool Add(const uint32_t id, std::unique_ptr<Frame> ptr) {
    if (Find(id)) {
      return false;
    }
    if (capacity_ > 0 && queue_.size() == capacity_) {
      map_.erase(queue_.front().first);
      queue_.pop();
    }
    queue_.push(std::make_pair(id, ptr.get()));
    map_[id] = std::move(ptr);
    return true;
  }

  void Clear() {
    capacity_ = 0;
    while (!queue_.empty()) {
      queue_.pop();
    }
    map_.clear();
  }

 public:
  std::size_t capacity_ = ConfigParam::FLAGS_max_history_frame_num;
  std::queue<std::pair<uint32_t, const Frame *>> queue_;
  std::unordered_map<uint32_t, std::unique_ptr<Frame>> map_;
};

// ===============================函数实现=================================

}  // namespace hqplanner

#endif