#ifndef HQPLANNER_TASKS_TASK_H_
#define HQPLANNER_TASKS_TASK_H_

#include <string>

#include "for_proto/planning_config.h"
#include "frame.h"
#include "reference_line_info.h"

namespace hqplanner {
namespace tasks {
using hqplanner::Frame;
using hqplanner::ReferenceLineInfo;
using hqplanner::forproto::PlanningConfig;

class Task {
 public:
  explicit Task(const std::string& name);
  virtual ~Task() = default;
  virtual const std::string& Name() const;

  virtual bool Execute(Frame* frame, ReferenceLineInfo* reference_line_info);

  virtual bool Init(const PlanningConfig& config);

 protected:
  bool is_init_ = false;
  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;

 private:
  const std::string name_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_TASK_H_
