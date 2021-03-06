#ifndef HQPLANNER_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_
#define HQPLANNER_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_

#include <vector>

// #include "modules/common/configs/proto/vehicle_config.pb.h"
#include "hqplanner/for_proto/vehicle_config.h"
// #include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "hqplanner/for_proto/dp_st_speed_config.h"
// #include "modules/planning/proto/planning_config.pb.h"
#include "hqplanner/for_proto/planning_config.h"

// #include "modules/common/configs/vehicle_config_helper.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
// #include "modules/common/status/status.h"
// #include "modules/planning/common/frame.h"
#include "hqplanner/frame.h"
// #include "modules/planning/common/path_decision.h"
#include "hqplanner/path_decision.h"
// #include "modules/planning/common/path_obstacle.h"
#include "hqplanner/path_obstacle.h"
// #include "modules/planning/common/speed/speed_data.h"
#include "hqplanner/speed/speed_data.h"
// #include "modules/planning/common/speed/st_point.h"
#include "hqplanner/speed/st_point.h"
// #include "modules/planning/tasks/dp_st_speed/dp_st_cost.h"
#include "hqplanner/tasks/dp_st_speed/dp_st_cost.h"
// #include "modules/planning/tasks/dp_st_speed/st_graph_point.h"
#include "hqplanner/tasks/dp_st_speed/st_graph_point.h"
// #include "modules/planning/tasks/st_graph/st_graph_data.h"
#include "hqplanner/tasks/st_graph/st_graph_data.h"

namespace hqplanner {
namespace tasks {

class DpStGraph {
 public:
  DpStGraph(const StGraphData& st_graph_data,
            const hqplanner::forproto::DpStSpeedConfig& dp_config,
            const std::vector<const hqplanner::PathObstacle*>& obstacles,
            const TrajectoryPoint& init_point,
            const SLBoundary& adc_sl_boundary);

  bool Search(SpeedData* const speed_data);

 private:
  bool InitCostTable();

  bool RetrieveSpeedProfile(SpeedData* const speed_data);

  bool CalculateTotalCost();
  void CalculateCostAt(const uint32_t r, const uint32_t c);

  float CalculateEdgeCost(const hqplanner::speed::STPoint& first,
                          const hqplanner::speed::STPoint& second,
                          const hqplanner::speed::STPoint& third,
                          const hqplanner::speed::STPoint& forth,
                          const float speed_limit);
  float CalculateEdgeCostForSecondCol(const uint32_t row,
                                      const float speed_limit);
  float CalculateEdgeCostForThirdCol(const uint32_t curr_r,
                                     const uint32_t pre_r,
                                     const float speed_limit);

  void GetRowRange(const StGraphPoint& point, int* highest_row,
                   int* lowest_row);

 private:
  const StGraphData& st_graph_data_;

  // dp st configuration
  hqplanner::forproto::DpStSpeedConfig dp_st_speed_config_;

  // obstacles based on the current reference line
  const std::vector<const hqplanner::PathObstacle*>& obstacles_;

  // vehicle configuration parameter
  const hqplanner::forproto::VehicleParam& vehicle_param_ =
      hqplanner::forproto::VehicleConfigHelper::GetConfig().vehicle_param;

  // initial status
  hqplanner::forproto::TrajectoryPoint init_point_;

  // cost utility with configuration;
  DpStCost dp_st_cost_;

  const SLBoundary& adc_sl_boundary_;

  float unit_s_ = 0.0;
  float unit_t_ = 0.0;

  // cost_table_[t][s]
  // row: s, col: t --- NOTICE: Please do NOT change.
  std::vector<std::vector<StGraphPoint>> cost_table_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_
