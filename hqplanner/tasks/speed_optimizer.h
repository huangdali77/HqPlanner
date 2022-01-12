#ifndef HQPLANNER_TASKS_SPEED_OPTIMIZER_H_
#define HQPLANNER_TASKS_SPEED_OPTIMIZER_H_

#include <string>
#include <vector>

#include "hqplanner/speed/speed_data.h"
#include "hqplanner/tasks/st_graph/st_graph_data.h"
#include "hqplanner/tasks/task.h"

namespace hqplanner {
namespace tasks {

class SpeedOptimizer : public Task {
 public:
  explicit SpeedOptimizer(const std::string& name);
  virtual ~SpeedOptimizer() = default;
  bool Execute(Frame* frame, ReferenceLineInfo* reference_line_info) override;

 protected:
  virtual bool Process(const SLBoundary& adc_sl_boundary,
                       const PathData& path_data,
                       const TrajectoryPoint& init_point,
                       const ReferenceLine& reference_line,
                       const SpeedData& reference_speed_data,
                       PathDecision* const path_decision,
                       SpeedData* const speed_data) = 0;

  //   void RecordSTGraphDebug(const StGraphData& st_graph_data,
  //                           planning_internal::STGraphDebug* stGraphDebug)
  //                           const;

  //   void RecordDebugInfo(const SpeedData& speed_data);
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_SPEED_OPTIMIZER_H_
