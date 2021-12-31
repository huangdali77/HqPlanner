#ifndef HQPLANNER_SUBSCRIBE_H_
#define HQPLANNER_SUBSCRIBE_H_

#include <ros/ros.h>

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
  PredictionObstacles GetPredictionObstacles() const {
    return prediction_obstacles_;
  }
  VehicleState GetVehicleState() const { return vehicle_state_; }
  std::vector<AnchorPoint> GetAnchorPoints() const { return anchor_points_; }
  std::list<ReferenceLine> GetReferenceLines() const {
    return reference_lines_;
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_prediction_obstacles_;
  ros::Subscriber sub_vehicle_state_;
  ros::Subscriber sub_anchor_points_;
  std::list PredictionObstacles prediction_obstacles_;
  VehicleState vehicle_state_;
  std::vector<AnchorPoint> anchor_points_;
  std::list<ReferenceLine> reference_lines_;
};

Subscribe::Subscribe() {
  sub_prediction_obstacles_ =
      nh_.subscribe("/prediction_obstacles", 10, callBack);
  sub_vehicle_state_ = nh_.subscribe("/vehicle_state", 10, callBack);
  sub_anchor_points_ = nh_.subscribe("/anchor_points", 10, callBack);
}
}  // namespace hqplanner
#endif