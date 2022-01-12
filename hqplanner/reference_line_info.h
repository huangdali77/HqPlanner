#ifndef HQPLANNER_REFERENCE_LINE_INFO_H_
#define HQPLANNER_REFERENCE_LINE_INFO_H_

#include <algorithm>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "for_proto/config_param.h"
#include "for_proto/pnc_point.h"
#include "for_proto/sl_boundary.h"
#include "for_proto/vehicle_config.h"
#include "for_proto/vehicle_state.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "path/path_data.h"
#include "path_decision.h"
#include "reference_line.h"
#include "speed/speed_data.h"
#include "trajectory/discretized_trajectory.h"
namespace hqplanner {
using hqplanner::PathDecision;
using hqplanner::forproto::PathPoint;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::SpeedPoint;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleConfig;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::forproto::VehicleParam;
using hqplanner::forproto::VehicleState;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedData;
using hqplanner::trajectory::DiscretizedTrajectory;
class ReferenceLineInfo {
  /* data */
 public:
  explicit ReferenceLineInfo(const VehicleState& vehicle_state,
                             const TrajectoryPoint& adc_planning_point,
                             const ReferenceLine& reference_line);

  bool Init(const std::vector<const Obstacle*>& obstacles);

  bool IsInited() const;

  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  PathObstacle* AddObstacle(const Obstacle* obstacle);

  PathDecision* path_decision();
  const PathDecision& path_decision() const;
  const ReferenceLine& reference_line() const;
  const TrajectoryPoint& AdcPlanningPoint() const;

  bool ReachedDestination() const;

  void SetTrajectory(const DiscretizedTrajectory& trajectory);

  const DiscretizedTrajectory& trajectory() const;
  double TrajectoryLength() const;

  double Cost() const { return cost_; }
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }
  void SetPriorityCost(double cost) { priority_cost_ = cost; }
  // For lattice planner'speed planning target
  // void SetStopPoint(const StopPoint& stop_point);
  void SetCruiseSpeed(double speed);
  // const PlanningTarget& planning_target() const { return planning_target_; }

  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
  bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;

  // planning_internal::Debug* mutable_debug() { return &debug_; }
  // const planning_internal::Debug& debug() const { return debug_; }
  // LatencyStats* mutable_latency_stats() { return &latency_stats_; }
  // const LatencyStats& latency_stats() const { return latency_stats_; }

  const PathData& path_data() const;
  const SpeedData& speed_data() const;
  PathData* mutable_path_data();
  SpeedData* mutable_speed_data();
  // aggregate final result together by some configuration
  bool CombinePathAndSpeedProfile(
      const double relative_time, const double start_s,
      DiscretizedTrajectory* discretized_trajectory);

  const SLBoundary& AdcSlBoundary() const;
  std::string PathSpeedDebugString() const;

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  bool IsChangeLanePath() const { return false; };

  /**
   * Check if the current reference line is the neighbor of the vehicle
   * current position
   */
  bool IsNeighborLanePath() const;

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDrivable(bool drivable);
  bool IsDrivable() const;

  // void ExportEngageAdvice(EngageAdvice* engage_advice) const;

  bool IsSafeToChangeLane() const { return is_safe_to_change_lane_; }

  // const hdmap::RouteSegments& Lanes() const;
  // const std::list<hdmap::Id> TargetLaneId() const;

  // void ExportDecision(DecisionResult* decision_result) const;

  // void SetJunctionRightOfWay(double junction_s, bool is_protected);

  // ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;

  // bool IsRightTurnPath() const;

  double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }
  void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
  }

  void set_is_on_reference_line() { is_on_reference_line_ = true; }

 private:
  const hqplanner::forproto::VehicleState vehicle_state_;
  const TrajectoryPoint adc_planning_point_;
  ReferenceLine reference_line_;

  double cost_ = 0.0;

  bool is_inited_ = false;

  bool is_drivable_ = true;

  PathDecision path_decision_;

  PathData path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  SLBoundary adc_sl_boundary_;

  // planning_internal::Debug debug_;
  // LatencyStats latency_stats_;

  // hdmap::RouteSegments lanes_;

  bool is_on_reference_line_ = true;

  bool is_safe_to_change_lane_ = false;

  // ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;

  ConfigParam config_param_;
};
// ===============函数实现==================================
ReferenceLineInfo::ReferenceLineInfo(const VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line) {}

bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param;
  const auto& path_point = adc_planning_point_.path_point;
  Vec2d position(path_point.x, path_point.y);
  Vec2d vec_to_center(
      (param.front_edge_to_center - param.back_edge_to_center) / 2.0,
      (param.left_edge_to_center - param.right_edge_to_center) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta));
  Box2d box(center, path_point.theta, param.length, param.width);

  // 获取自车的sl_boundary
  if (!reference_line_.GetSLBoundary(box, &adc_sl_boundary_)) {
    return false;
  }
  if (adc_sl_boundary_.end_s < 0 ||
      adc_sl_boundary_.start_s > reference_line_.Length()) {
    assert(0);
    // AWARN << "Vehicle SL " << adc_sl_boundary_.ShortDebugString()
    //       << " is not on reference line:[0, " << reference_line_.Length()
    //       << "]";
  }
  constexpr double kOutOfReferenceLineL = 10.0;  // in meters
  if (adc_sl_boundary_.start_l > kOutOfReferenceLineL ||
      adc_sl_boundary_.end_l < -kOutOfReferenceLineL) {
    is_on_reference_line_ = false;
    // AERROR << "Ego vehicle is too far away from reference line.";
    return false;
  }

  if (!AddObstacles(obstacles)) {
    return false;
  }

  // if (hdmap::GetSpeedControls()) {
  //   auto* speed_controls = hdmap::GetSpeedControls();
  //   for (const auto& speed_control : speed_controls->speed_control()) {
  //     reference_line_.AddSpeedLimit(speed_control);
  //   }
  // }

  // set lattice planning target speed limit;
  // SetCruiseSpeed(FLAGS_default_cruise_speed);
  // is_safe_to_change_lane_ = CheckChangeLane();
  is_inited_ = true;
  return true;
}

bool ReferenceLineInfo::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto* obstacle : obstacles) {
    if (!AddObstacle(obstacle)) {
      return false;
    }
  }
  return true;
}

PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  if (!obstacle) {
    // AERROR << "The provided obstacle is empty";
    return nullptr;
  }
  auto* path_obstacle = path_decision_.AddPathObstacle(PathObstacle(obstacle));
  if (!path_obstacle) {
    // AERROR << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(),
                                     &perception_sl)) {
    // AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
    return path_obstacle;
  }
  path_obstacle->SetPerceptionSlBoundary(perception_sl);

  // if (IsUnrelaventObstacle(path_obstacle)) {
  //   ObjectDecisionType ignore;
  //   ignore.mutable_ignore();
  //   path_decision_.AddLateralDecision("reference_line_filter",
  //   obstacle->Id(),
  //                                     ignore);
  //   path_decision_.AddLongitudinalDecision("reference_line_filter",
  //                                          obstacle->Id(), ignore);
  //   ADEBUG << "NO build reference line st boundary. id:" << obstacle->Id();
  // // } else {
  //   ADEBUG << "build reference line st boundary. id:" << obstacle->Id();
  path_obstacle->BuildReferenceLineStBoundary(reference_line_,
                                              adc_sl_boundary_.start_s);

  // ADEBUG << "reference line st boundary: "
  //        << path_obstacle->reference_line_st_boundary().min_t() << ", "
  //        << path_obstacle->reference_line_st_boundary().max_t()
  //        << ", s_max: " <<
  //        path_obstacle->reference_line_st_boundary().max_s()
  //        << ", s_min: " <<
  //        path_obstacle->reference_line_st_boundary().min_s();
  // }
  return path_obstacle;
}

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }
const PathData& ReferenceLineInfo::path_data() const { return path_data_; }

const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }

PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }

SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }

bool ReferenceLineInfo::IsInited() const { return is_inited_; }

PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }

const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}
const TrajectoryPoint& ReferenceLineInfo::AdcPlanningPoint() const {
  return adc_planning_point_;
}

const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {
  return adc_sl_boundary_;
}
void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}
double ReferenceLineInfo::TrajectoryLength() const {
  const auto& tps = discretized_trajectory_.trajectory_points();
  if (tps.empty()) {
    return 0.0;
  }
  return tps.back().path_point.s;
}

bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  assert(ptr_discretized_trajectory != nullptr);
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion =
      config_param_.FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution =
      config_param_.FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec =
      config_param_.FLAGS_trajectory_time_high_density_period;
  if (path_data_.discretized_path().NumOfPoints() == 0) {
    // AWARN << "path data is empty";
    return false;
  }
  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      // AERROR << "Fail to get speed point with relative time " <<
      // cur_rel_time;
      return false;
    }

    if (speed_point.s > path_data_.discretized_path().Length()) {
      break;
    }
    PathPoint path_point;
    if (!path_data_.GetPathPointWithPathS(speed_point.s, &path_point)) {
      // AERROR << "Fail to get path data with s " << speed_point.s()
      //        << "path total length " <<
      //        path_data_.discretized_path().Length();
      return false;
    }
    path_point.s = path_point.s + start_s;

    TrajectoryPoint trajectory_point;
    // trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.path_point = path_point;
    trajectory_point.v = speed_point.v;
    trajectory_point.a = speed_point.a;
    trajectory_point.relative_time = speed_point.t + relative_time;
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}
bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }
}  // namespace hqplanner

#endif