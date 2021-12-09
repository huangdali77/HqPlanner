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

#include "for_proto/adc_trajectory.h";
#include "for_proto/perception_obstacle.h"
#include "for_proto/pnc_point.h"
#include "for_proto/prediction_obstacle.h"
#include "for_proto/vehicle_state.h"
#include "math/vec2d.h"
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
  // const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
  //                                             const common::math::Box2d
  //                                             &box);

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

  ADCTrajectory trajectory_;  // last published trajectory

  ReferenceLineProvider *reference_line_provider_ = nullptr;
};

// ===============================函数实现=================================

}  // namespace hqplanner

#endif