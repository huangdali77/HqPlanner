#ifndef HQPLANNER_PATH_OBSTACLE_H_
#define HQPLANNER_PATH_OBSTACLE_H_
// #include "hqplanner/for_proto/vehicle_config_helper.h"
#include <list>
#include <string>
#include <unordered_map>
#include <vector>

#include "for_proto/config_param.h"
#include "for_proto/decision.h"
#include "for_proto/perception_obstacle.h"
#include "for_proto/sl_boundary.h"
#include "hqplanner/for_proto/vehicle_config.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/speed/st_boundary.h"
#include "hqplanner/speed/st_point.h"
#include "math/vec2d.h"
#include "obstacle.h"
#include "reference_line.h"

namespace hqplanner {

using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::ObjectDecisionType;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::forproto::VehicleParam;
using hqplanner::math::Box2d;
using hqplanner::math::Vec2d;
using hqplanner::speed::StBoundary;
using hqplanner::speed::STPoint;
class PathObstacle {
 public:
  PathObstacle() = default;
  explicit PathObstacle(const Obstacle* obstacle);

  const std::string& Id() const;

  const Obstacle* obstacle() const;

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const hqplanner::forproto::ObjectDecisionType& LateralDecision() const;

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const hqplanner::forproto::ObjectDecisionType& LongitudinalDecision() const;

  const SLBoundary& PerceptionSLBoundary() const;

  const StBoundary& reference_line_st_boundary() const;

  const StBoundary& st_boundary() const;

  const std::vector<std::string>& decider_tags() const;

  const std::vector<hqplanner::forproto::ObjectDecisionType>& decisions() const;

  void AddLongitudinalDecision(
      const std::string& decider_tag,
      const hqplanner::forproto::ObjectDecisionType& decision);

  void AddLateralDecision(
      const std::string& decider_tag,
      const hqplanner::forproto::ObjectDecisionType& decision);

  bool HasLateralDecision() const;

  void SetStBoundary(const StBoundary& boundary);

  void SetStBoundaryType(const StBoundary::BoundaryType type);

  void EraseStBoundary();

  void SetReferenceLineStBoundary(const StBoundary& boundary);

  void SetReferenceLineStBoundaryType(const StBoundary::BoundaryType type);

  void EraseReferenceLineStBoundary();

  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;

  void BuildReferenceLineStBoundary(ReferenceLine& reference_line,
                                    const double adc_start_s);

  void SetPerceptionSlBoundary(const SLBoundary& sl_boundary);

  /**
   * @brief check if a ObjectDecisionType is a longitudinal decision.
   **/
  static bool IsLongitudinalDecision(
      const hqplanner::forproto::ObjectDecisionType& decision);

  /**
   * @brief check if a ObjectDecisionType is a lateral decision.
   **/
  static bool IsLateralDecision(
      const hqplanner::forproto::ObjectDecisionType& decision);

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }
  double MinRadiusStopDistance(
      const hqplanner::forproto::VehicleParam& vehicle_param) const;

 private:
  static hqplanner::forproto::ObjectDecisionType MergeLongitudinalDecision(
      const hqplanner::forproto::ObjectDecisionType& lhs,
      const hqplanner::forproto::ObjectDecisionType& rhs);
  static hqplanner::forproto::ObjectDecisionType MergeLateralDecision(
      const hqplanner::forproto::ObjectDecisionType& lhs,
      const hqplanner::forproto::ObjectDecisionType& rhs);
  bool BuildTrajectoryStBoundary(ReferenceLine& reference_line,
                                 const double adc_start_s,
                                 StBoundary* const st_boundary);
  bool IsValidObstacle(const PerceptionObstacle& perception_obstacle);
  std::string id_;
  const Obstacle* obstacle_ = nullptr;
  std::vector<hqplanner::forproto::ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;

  SLBoundary perception_sl_boundary_;
  StBoundary reference_line_st_boundary_;
  StBoundary st_boundary_;

  hqplanner::forproto::ObjectDecisionType lateral_decision_;
  hqplanner::forproto::ObjectDecisionType longitudinal_decision_;
  // VehicleParam adc_param_;
  bool is_blocking_obstacle_ = false;
  double min_radius_stop_distance_ = -1.0;
  // ConfigParam config_param_;
};
// =========================函数实现==============================

// namespace {
// const double kStBoundaryDeltaS = 0.2;        // meters
// const double kStBoundarySparseDeltaS = 1.0;  // meters
// const double kStBoundaryDeltaT = 0.05;       // seconds
// }  // namespace

// const Obstacle* PathObstacle::obstacle() const { return obstacle_; }
// const std::string& PathObstacle::Id() const { return id_; }

// PathObstacle::PathObstacle(const Obstacle* obstacle) : obstacle_(obstacle) {
//   assert(obstacle != nullptr);
//   id_ = obstacle_->Id();
// }

// void PathObstacle::SetPerceptionSlBoundary(const SLBoundary& sl_boundary) {
//   perception_sl_boundary_ = sl_boundary;
// }

// void PathObstacle::BuildReferenceLineStBoundary(ReferenceLine&
// reference_line,
//                                                 const double adc_start_s) {
//   // const VehicleParam adc_param;
//   const auto& adc_param =
//       VehicleConfigHelper::instance()->GetConfig().vehicle_param;
//   const double adc_width = adc_param.width;
//   if (obstacle_->IsStatic() ||
//       obstacle_->Trajectory().trajectory_point.empty()) {
//     std::vector<std::pair<STPoint, STPoint>> point_pairs;
//     double start_s = perception_sl_boundary_.start_s;
//     double end_s = perception_sl_boundary_.end_s;
//     if (end_s - start_s < kStBoundaryDeltaS) {
//       end_s = start_s + kStBoundaryDeltaS;
//     }

//     point_pairs.emplace_back(STPoint(start_s - adc_start_s, 0.0),
//                              STPoint(end_s - adc_start_s, 0.0));
//     point_pairs.emplace_back(
//         STPoint(start_s - adc_start_s, ConfigParam::FLAGS_st_max_t),
//         STPoint(end_s - adc_start_s, ConfigParam::FLAGS_st_max_t));
//     reference_line_st_boundary_ = StBoundary(point_pairs);
//   } else {
//     BuildTrajectoryStBoundary(reference_line, adc_start_s,
//                               &reference_line_st_boundary_);
//   }
// }

// bool PathObstacle::BuildTrajectoryStBoundary(ReferenceLine& reference_line,
//                                              const double adc_start_s,
//                                              StBoundary* const st_boundary) {
//   const auto& object_id = obstacle_->Id();
//   const auto& perception = obstacle_->Perception();
//   if (!IsValidObstacle(perception)) {
//     // AERROR << "Fail to build trajectory st boundary because object is not
//     "
//     //           "valid. PerceptionObstacle: "
//     //        << perception.DebugString();
//     return false;
//   }
//   const double object_width = perception.width;
//   const double object_length = perception.length;
//   const auto& trajectory_points = obstacle_->Trajectory().trajectory_point;
//   if (trajectory_points.empty()) {
//     // AWARN << "object " << object_id << " has no trajectory points";
//     return false;
//   }

//   const auto& adc_param =
//       VehicleConfigHelper::instance()->GetConfig().vehicle_param;
//   const double adc_length = adc_param.length;
//   const double adc_half_length = adc_length / 2.0;
//   const double adc_width = adc_param.width;

//   // Box2d min_box({0, 0}, 1.0, 1.0, 1.0);
//   // Box2d max_box({0, 0}, 1.0, 1.0, 1.0);
//   std::vector<std::pair<STPoint, STPoint>> polygon_points;

//   SLBoundary last_sl_boundary;
//   int last_index = 0;

//   for (int i = 1; i < trajectory_points.size(); ++i) {
//     const auto& first_traj_point = trajectory_points[i - 1];
//     const auto& second_traj_point = trajectory_points[i];
//     const auto& first_point = first_traj_point.path_point;
//     const auto& second_point = second_traj_point.path_point;

//     double total_length =
//         object_length + std::hypot(first_point.x - second_point.x,
//                                    first_point.y - second_point.y);

//     Vec2d center((first_point.x + second_point.x) / 2.0,
//                  (first_point.y + second_point.y) / 2.0);
//     Box2d object_moving_box(center, first_point.theta, total_length,
//                             object_width);
//     SLBoundary object_boundary;
//     // NOTICE: this method will have errors when the reference line is not
//     // straight. Need double loop to cover all corner cases.

//     const double distance_dx = trajectory_points[last_index].path_point.x -
//                                trajectory_points[i].path_point.x;
//     const double distance_dy = trajectory_points[last_index].path_point.y -
//                                trajectory_points[i].path_point.y;
//     const double distance_xy = std::hypot(distance_dx, distance_dy);

//     if (last_sl_boundary.start_l > distance_xy ||
//         last_sl_boundary.end_l < -distance_xy) {
//       continue;
//     }

//     const double mid_s =
//         (last_sl_boundary.start_s + last_sl_boundary.end_s) / 2.0;
//     const double start_s = std::fmax(0.0, mid_s - 2.0 * distance_xy);
//     const double end_s = (i == 1) ? reference_line.Length()
//                                   : std::fmin(reference_line.Length(),
//                                               mid_s + 2.0 * distance_xy);

//     if (!reference_line.GetApproximateSLBoundary(object_moving_box,
//                                                  &object_boundary)) {
//       // AERROR << "failed to calculate boundary";
//       return false;
//     }

//     // update history record
//     last_sl_boundary = object_boundary;
//     last_index = i;

//     // skip if object is entirely on one side of reference line.
//     constexpr double kSkipLDistanceFactor = 0.4;
//     const double skip_l_distance =
//         (object_boundary.end_s - object_boundary.start_s) *
//             kSkipLDistanceFactor +
//         adc_width / 2.0;

//     if (std::fmin(object_boundary.start_l, object_boundary.end_l) >
//             skip_l_distance ||
//         std::fmax(object_boundary.start_l, object_boundary.end_l) <
//             -skip_l_distance) {
//       continue;
//     }

//     if (object_boundary.end_s < 0) {  // skip if behind reference line
//       continue;
//     }
//     constexpr double kSparseMappingS = 20.0;
//     const double st_boundary_delta_s =
//         (std::fabs(object_boundary.start_s - adc_start_s) > kSparseMappingS)
//             ? kStBoundarySparseDeltaS
//             : kStBoundaryDeltaS;
//     const double object_s_diff =
//         object_boundary.end_s - object_boundary.start_s;
//     if (object_s_diff < st_boundary_delta_s) {
//       continue;
//     }
//     const double delta_t =
//         second_traj_point.relative_time - first_traj_point.relative_time;
//     double low_s = std::max(object_boundary.start_s - adc_half_length, 0.0);
//     bool has_low = false;
//     double high_s = std::min(object_boundary.end_s + adc_half_length,
//                              ConfigParam::FLAGS_st_max_s);
//     bool has_high = false;
//     while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high)) {
//       if (!has_low) {
//         auto low_ref = reference_line.GetReferencePoint(low_s);
//         Box2d low_ref_box({low_ref.x, low_ref.y}, low_ref.heading,
//         adc_length,
//                           adc_width);
//         has_low = object_moving_box.HasOverlap(low_ref_box);
//         low_s += st_boundary_delta_s;
//       }
//       if (!has_high) {
//         auto high_ref = reference_line.GetReferencePoint(high_s);
//         Box2d high_ref_box({high_ref.x, high_ref.y}, high_ref.heading,
//                            adc_length, adc_width);
//         has_high = object_moving_box.HasOverlap(high_ref_box);
//         high_s -= st_boundary_delta_s;
//       }
//     }
//     if (has_low && has_high) {
//       low_s -= st_boundary_delta_s;
//       high_s += st_boundary_delta_s;
//       double low_t =
//           (first_traj_point.relative_time +
//            std::fabs((low_s - object_boundary.start_s) / object_s_diff) *
//                delta_t);
//       polygon_points.emplace_back(
//           std::make_pair(STPoint{low_s - adc_start_s, low_t},
//                          STPoint{high_s - adc_start_s, low_t}));
//       double high_t =
//           (first_traj_point.relative_time +
//            std::fabs((high_s - object_boundary.start_s) / object_s_diff) *
//                delta_t);
//       if (high_t - low_t > 0.05) {
//         polygon_points.emplace_back(
//             std::make_pair(STPoint{low_s - adc_start_s, high_t},
//                            STPoint{high_s - adc_start_s, high_t}));
//       }
//     }
//   }
//   if (!polygon_points.empty()) {
//     std::sort(polygon_points.begin(), polygon_points.end(),
//               [](const std::pair<STPoint, STPoint>& a,
//                  const std::pair<STPoint, STPoint>& b) {
//                 return a.first.t() < b.first.t();
//               });
//     auto last = std::unique(polygon_points.begin(), polygon_points.end(),
//                             [](const std::pair<STPoint, STPoint>& a,
//                                const std::pair<STPoint, STPoint>& b) {
//                               return std::fabs(a.first.t() - b.first.t()) <
//                                      kStBoundaryDeltaT;
//                             });
//     polygon_points.erase(last, polygon_points.end());
//     if (polygon_points.size() > 2) {
//       *st_boundary = StBoundary(polygon_points);
//     }
//   } else {
//     return false;
//   }
//   return true;
// }

// const SLBoundary& PathObstacle::PerceptionSLBoundary() const {
//   return perception_sl_boundary_;
// }

// void PathObstacle::SetStBoundary(const StBoundary& boundary) {
//   st_boundary_ = boundary;
// }

// void PathObstacle::SetStBoundaryType(const StBoundary::BoundaryType type) {
//   st_boundary_.SetBoundaryType(type);
// }

// void PathObstacle::EraseStBoundary() { st_boundary_ = StBoundary(); }

// void PathObstacle::SetReferenceLineStBoundary(const StBoundary& boundary) {
//   reference_line_st_boundary_ = boundary;
// }

// void PathObstacle::SetReferenceLineStBoundaryType(
//     const StBoundary::BoundaryType type) {
//   reference_line_st_boundary_.SetBoundaryType(type);
// }

// void PathObstacle::EraseReferenceLineStBoundary() {
//   reference_line_st_boundary_ = StBoundary();
// }

// bool PathObstacle::IsValidObstacle(
//     const PerceptionObstacle& perception_obstacle) {
//   const double object_width = perception_obstacle.width;
//   const double object_length = perception_obstacle.length;

//   const double kMinObjectDimension = 1.0e-6;
//   return !std::isnan(object_width) && !std::isnan(object_length) &&
//          object_width > kMinObjectDimension &&
//          object_length > kMinObjectDimension;
// }

}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_PATH_OBSTACLE_H_
