#ifndef HQPLANER_EM_PLANNER_H_
#define HQPLANER_EM_PLANNER_H_

#include "hqplanner/for_proto/pnc_point.h"
// #include "modules/common/status/status.h"
// #include "modules/planning/common/frame.h"
#include "hqplanner/for_proto/planning_config.h"
#include "hqplanner/frame.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace hqplanner {
namespace tasks {
using hqplanner::Frame;
using hqplanner::ReferenceLineInfo;
using hqplanner::forproto::PlanningConfig;
using hqplanner::forproto::TrajectoryPoint;
/**
 * @class Planner
 * @brief Planner is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.
 */
class Planner {
 public:
  /**
   * @brief Constructor
   */
  Planner() = default;

  /**
   * @brief Destructor
   */
  virtual ~Planner() = default;

  virtual bool Init(const PlanningConfig& config) = 0;

  /**
   * @brief Compute trajectories for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual bool Plan(const TrajectoryPoint& planning_init_point,
                    Frame* frame) = 0;

  /**
   * @brief Compute a trajectory for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual bool PlanOnReferenceLine(const TrajectoryPoint& planning_init_point,
                                   Frame* frame,
                                   ReferenceLineInfo* reference_line_info) = 0;
};

}  // namespace tasks
}  // namespace hqplanner

#endif /* MODULES_PLANNING_PLANNER_PLANNER_H_ */
