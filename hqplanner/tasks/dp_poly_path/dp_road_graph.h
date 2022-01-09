#ifndef HQPLANNER_TASKS_DP_ROAD_GRAPH_H_
#define HQPLANNER_TASKS_DP_ROAD_GRAPH_H_

#include <limits>
#include <list>
#include <vector>

#include "hqplanner/for_proto/decision.h"
#include "hqplanner/for_proto/dp_poly_path_config.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/math/curve1d/quintic_polynomial_curve1d.h"
#include "hqplanner/path/path_data.h"
#include "hqplanner/path_decision.h"
#include "hqplanner/path_obstacle.h"
#include "hqplanner/reference_line_info.h"
#include "hqplanner/speed/speed_data.h"
#include "hqplanner/tasks/dp_poly_path/trajectory_cost.h"
#include "hqplanner/trajectory/discretized_trajectory.h"
namespace hqplanner {
namespace tasks {
using hqplanner::PathObstacle;
using hqplanner::ReferenceLineInfo;
using hqplanner::forproto::FrenetFramePoint;
using hqplanner::forproto::ObjectSidePass;
using hqplanner::forproto::SLPoint;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedData;
using hqplanner::tasks::DpPolyPathConfig;
class DPRoadGraph {
 public:
  explicit DPRoadGraph(const DpPolyPathConfig &config,
                       const ReferenceLineInfo &reference_line_info,
                       const SpeedData &speed_data);

  ~DPRoadGraph() = default;

  bool FindPathTunnel(const TrajectoryPoint &init_point,
                      const std::vector<const PathObstacle *> &obstacles,
                      PathData *const path_data);

  // void SetDebugLogger(apollo::planning_internal::Debug *debug) {
  //   planning_debug_ = debug;
  // }

 private:
  /**
   * an private inner struct for the dp algorithm
   */
  struct DPRoadGraphNode {
   public:
    DPRoadGraphNode() = default;

    DPRoadGraphNode(const SLPoint point_sl, const DPRoadGraphNode *node_prev)
        : sl_point(point_sl), min_cost_prev_node(node_prev) {}

    DPRoadGraphNode(const SLPoint point_sl, const DPRoadGraphNode *node_prev,
                    const ComparableCost &cost)
        : sl_point(point_sl), min_cost_prev_node(node_prev), min_cost(cost) {}

    void UpdateCost(const DPRoadGraphNode *node_prev,
                    const QuinticPolynomialCurve1d &curve,
                    const ComparableCost &cost) {
      if (cost <= min_cost) {
        min_cost = cost;
        min_cost_prev_node = node_prev;
        min_cost_curve = curve;
      }
    }

    SLPoint sl_point;
    const DPRoadGraphNode *min_cost_prev_node = nullptr;
    ComparableCost min_cost = {true, true, true,
                               std::numeric_limits<float>::infinity(),
                               std::numeric_limits<float>::infinity()};
    QuinticPolynomialCurve1d min_cost_curve;
  };

  bool GenerateMinCostPath(const std::vector<const PathObstacle *> &obstacles,
                           std::vector<DPRoadGraphNode> *min_cost_path);

  bool SamplePathWaypoints(const TrajectoryPoint &init_point,
                           std::vector<std::vector<SLPoint>> *const points);

  bool CalculateFrenetPoint(const TrajectoryPoint &traj_point,
                            FrenetFramePoint *const frenet_frame_point);

  bool IsValidCurve(const QuinticPolynomialCurve1d &curve) const;

  void GetCurveCost(TrajectoryCost trajectory_cost,
                    const QuinticPolynomialCurve1d &curve, const float start_s,
                    const float end_s, const uint32_t curr_level,
                    const uint32_t total_level, ComparableCost *cost);

  void UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,
                  const uint32_t level, const uint32_t total_level,
                  TrajectoryCost *trajectory_cost, DPRoadGraphNode *front,
                  DPRoadGraphNode *cur_node);
  bool HasSidepass();

 private:
  DpPolyPathConfig config_;
  TrajectoryPoint init_point_;
  const ReferenceLineInfo &reference_line_info_;
  const ReferenceLine &reference_line_;
  SpeedData speed_data_;
  SLPoint init_sl_point_;
  FrenetFramePoint init_frenet_frame_point_;
  // apollo::planning_internal::Debug *planning_debug_ = nullptr;

  ObjectSidePass sidepass_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_DP_POLY_PATH_DP_ROAD_GRAPH_H_
