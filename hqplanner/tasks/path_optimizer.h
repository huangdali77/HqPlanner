#ifndef HQPLANNER_TASKS_PATH_OPTIMIZER_H_
#define HQPLANNER_TASKS_PATH_OPTIMIZER_H_

#include <string>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/reference_line.h"
#include "hqplanner/speed/speed_data.h"
#include "hqplanner/tasks/task.h"
namespace hqplanner {
namespace tasks {
using hqplanner::Frame;
using hqplanner::ReferenceLine;
using hqplanner::ReferenceLineInfo;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::speed::SpeedData;
class PathOptimizer : public Task {
 public:
  explicit PathOptimizer(const std::string &name);
  virtual ~PathOptimizer() = default;
  bool Execute(Frame *frame, ReferenceLineInfo *reference_line_info) override;

 protected:
  virtual bool Process(const SpeedData &speed_data,
                       const ReferenceLine &reference_line,
                       const TrajectoryPoint &init_point,
                       PathData *const path_data) = 0;

  //   void RecordDebugInfo(const PathData &path_data);
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_PATH_OPTIMIZER_H_
