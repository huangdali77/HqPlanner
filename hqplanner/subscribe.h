#ifndef HQPLANNER_SUBSCRIBE_H_
#define HQPLANNER_SUBSCRIBE_H_

// #include <ros/ros.h>

#include <list>
#include <vector>

#include "for_proto/perception_obstacle.h"
#include "for_proto/pnc_point.h"
#include "for_proto/prediction_obstacle.h"
#include "for_proto/vehicle_state.h"
#include "reference_line.h"
namespace hqplanner {
using hqplanner::ReferenceLine;
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::PerceptionObstacles;
using hqplanner::forproto::PredictionObstacle;
using hqplanner::forproto::PredictionObstacles;
using hqplanner::forproto::VehicleState;
class Subscribe {
 public:
  Subscribe();
  // static bool UpdateSubscribeInfo(ros::NodeHandle &nh_);

  PredictionObstacles GetPredictionObstacles() const {
    return prediction_obstacles_;
  }
  VehicleState GetVehicleState() const { return vehicle_state_; }
  std::vector<std::vector<AnchorPoint>> GetAnchorPoints() const {
    return anchor_points_;
  }
  std::list<ReferenceLine> GetReferenceLines() const {
    return reference_lines_;
  }

 private:
  // static void CallBackPredictionObstacles(
  //     const ros_msg::PredictionObstacles &prediction_obstacles_msg);

  // static void CallBackVehicleState(
  //     const ros_msg::VehicleState &vehicle_state_msg);
  // static void CallBackAnchorPoints(
  //     const ros_msg::AnchorPoints &anchor_points_msg);

 public:
  static PredictionObstacles prediction_obstacles_;
  static VehicleState vehicle_state_;
  static std::vector<std::vector<AnchorPoint>> anchor_points_;
  static std::list<ReferenceLine> reference_lines_;

 private:
  // static ros::Subscriber sub_prediction_obstacles_;
  // static ros::Subscriber sub_vehicle_state_;
  // static ros::Subscriber sub_anchor_points_;
};

PredictionObstacles Subscribe::prediction_obstacles_;
VehicleState Subscribe::vehicle_state_;
std::vector<std::vector<AnchorPoint>> Subscribe::anchor_points_;
std::list<ReferenceLine> Subscribe::reference_lines_;

// bool Subscribe::UpdateSubscribeInfo(ros::NodeHandle &nh_) {
//   prediction_obstacles_.prediction_obstacle.clear();
//   anchor_points_.clear();
//   reference_lines_.clear();

//   sub_prediction_obstacles_ =
//       nh_.subscribe("/prediction_obstacles", 10,
//       CallBackPredictionObstacles);
//   sub_vehicle_state_ =
//       nh_.subscribe("/vehicle_state", 10, CallBackVehicleState);
//   sub_anchor_points_ =
//       nh_.subscribe("/anchor_points", 10, CallBackAnchorPoints);
// }

// void Subscribe::CallBackPredictionObstacles(
//     const ros_msg::PredictionObstacles &prediction_obstacles_msg) {
//   // prediction_obstacles_.start_timestamp =
//   // prediction_obstacles_.end_timestamp =

//   // for(){
//   //   prediction_obstacles_.prediction_obstacle.push_back();
//   // }
// }
// void Subscribe::CallBackAnchorPoints(
//     const ros_msg::AnchorPoints &anchor_points_msg) {
//   // for (int i = 0;i<;++i){
//   //   for(){
//   //     anchor_points_[i].push_back();
//   //   }
//   // }
// }
// void Subscribe::CallBackVehicleState(
//     const ros_msg::VehicleState &vehicle_state_msg) {
//   // vehicle_state_.x = ;
//   // vehicle_state_.y = ;
// }

}  // namespace hqplanner
#endif