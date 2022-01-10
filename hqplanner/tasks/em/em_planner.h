#ifndef HQPLANNER_TASKS_EM_EM_PLANNER_H_
#define HQPLANNER_TASKS_EM_EM_PLANNER_H_

#include <memory>
#include <string>
#include <vector>

#include "hqplanner/math/curve1d/quintic_polynomial_curve1d.h"
#include "hqplanner/reference_line.h"
#include "hqplanner/reference_line_info.h"
#include "hqplanner/tasks/em/planner.h"
// #include "modules/planning/proto/planning.pb.h"

#include "hqplanner/tasks/task.h"
/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace hqplanner {
namespace tasks {
using hqplanner::forproto::ReferencePoint;
using hqplanner::math::QuinticPolynomialCurve1d;
/**
 * @class EMPlanner
 * @brief EMPlanner is an expectation maximization planner.
 */

class EMPlanner : public Planner {
 public:
  /**
   * @brief Constructor
   */
  EMPlanner() = default;

  /**
   * @brief Destructor
   */
  virtual ~EMPlanner() = default;

  bool Init(const PlanningConfig& config) override;
  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  bool Plan(const TrajectoryPoint& planning_init_point, Frame* frame) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  bool PlanOnReferenceLine(const TrajectoryPoint& planning_init_point,
                           Frame* frame,
                           ReferenceLineInfo* reference_line_info) override;

 private:
  //   void RegisterTasks();

  std::vector<SpeedPoint> GenerateInitSpeedProfile(
      const TrajectoryPoint& planning_init_point,
      const ReferenceLineInfo* reference_line_info);

  std::vector<SpeedPoint> DummyHotStart(
      const TrajectoryPoint& planning_init_point);

  std::vector<SpeedPoint> GenerateSpeedHotStart(
      const TrajectoryPoint& planning_init_point);

  void GenerateFallbackPathProfile(const ReferenceLineInfo* reference_line_info,
                                   PathData* path_data);

  void GenerateFallbackSpeedProfile(
      const ReferenceLineInfo* reference_line_info, SpeedData* speed_data);

  SpeedData GenerateStopProfile(const double init_speed,
                                const double init_acc) const;

  SpeedData GenerateStopProfileFromPolynomial(const double init_speed,
                                              const double init_acc) const;

  bool IsValidProfile(const QuinticPolynomialCurve1d& curve) const;

  void RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info);

  void RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                       const std::string& name, const double time_diff_ms);

  //   apollo::util::Factory<TaskType, Task> task_factory_;
  std::vector<std::unique_ptr<Task>> tasks_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_PLANNER_EM_EM_PLANNER_H_
