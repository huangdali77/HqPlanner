#ifndef HQPLANNER_REFERENCE_LINE_INFO_H_
#define HQPLANNER_REFERENCE_LINE_INFO_H_

#include <algorithm>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
class ReferenceLineInfo {
  /* data */
 public:
  ReferenceLineInfo(/* args */);
  ~ReferenceLineInfo();

 private:
  double cost_ = 0.0;

  bool is_inited_ = false;

  bool is_drivable_ = true;

  PathDecision path_decision_;

  PathData path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  SLBoundary adc_sl_boundary_;

  planning_internal::Debug debug_;
  LatencyStats latency_stats_;

  hdmap::RouteSegments lanes_;

  bool is_on_reference_line_ = false;

  bool is_safe_to_change_lane_ = false;

  ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;
};

ReferenceLineInfo::ReferenceLineInfo(/* args */) {}

ReferenceLineInfo::~ReferenceLineInfo() {}

#endif