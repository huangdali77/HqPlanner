#ifndef HQPLANNER_REFERENCE_LINE_PROVIDER_H_
#define HQPLANNER_REFERENCE_LINE_PROVIDER_H_
#include <list>

#include "for_proto/config_param.h"
#include "for_proto/vehicle_state.h"
#include "reference_line.h"
#include "subscribe.h"
namespace hqplanner {
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::VehicleState;

class ReferenceLineProvider {
 public:
  explicit ReferenceLineProvider(
      std::vector<std::vector<AnchorPoint>> anchor_points);
  bool AddReferenceLine(ReferenceLine ref_line) {
    reference_lines_.emplace_back(ref_line);
  }

  bool GetReferenceLines(std::list<ReferenceLine>* reference_lines);
  static double LookForwardDistance(const VehicleState& state);

 private:
  std::list<ReferenceLine> reference_lines_;
  // Subscribe subscribe_info_;
  std::vector<std::vector<AnchorPoint>> anchor_points_;
};

// ==========================函数实现===========================

ReferenceLineProvider::ReferenceLineProvider(
    std::vector<std::vector<AnchorPoint>> anchor_points)
    : anchor_points_(anchor_points) {
  for (auto anchor_points : anchor_points_) {
    ReferenceLine ref(anchor_points);
    AddReferenceLine(ref);
  }
}

bool ReferenceLineProvider::GetReferenceLines(
    std::list<ReferenceLine>* reference_lines) {
  if (reference_lines_.empty()) {
    return false;
  }
  for (auto ref_line : reference_lines_) {
    reference_lines->push_back(ref_line);
  }
  return true;
}

double ReferenceLineProvider::LookForwardDistance(const VehicleState& state) {
  auto forward_distance =
      state.linear_velocity * ConfigParam::FLAGS_look_forward_time_sec;

  if (forward_distance > ConfigParam::FLAGS_look_forward_short_distance) {
    return ConfigParam::FLAGS_look_forward_long_distance;
  }

  return ConfigParam::FLAGS_look_forward_short_distance;
}

}  // namespace hqplanner

#endif