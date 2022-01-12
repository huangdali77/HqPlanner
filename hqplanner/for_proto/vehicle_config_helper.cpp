// #include "modules/common/configs/vehicle_config_helper.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"

#include <algorithm>
#include <cmath>

// #include "modules/common/configs/config_gflags.h"
// #include "modules/common/util/file.h"

namespace hqplanner {
namespace forproto {

VehicleConfig VehicleConfigHelper::vehicle_config_;
bool VehicleConfigHelper::is_init_ = true;

VehicleConfigHelper::VehicleConfigHelper() {}

void VehicleConfigHelper::Init(const VehicleConfig &vehicle_params) {
  vehicle_config_ = vehicle_params;
  is_init_ = true;
}

const VehicleConfig &VehicleConfigHelper::GetConfig() {
  return vehicle_config_;
}

double VehicleConfigHelper::MinSafeTurnRadius() {
  const auto &param = vehicle_config_.vehicle_param;
  double lat_edge_to_center =
      std::max(param.left_edge_to_center, param.right_edge_to_center);
  double lon_edge_to_center =
      std::max(param.front_edge_to_center, param.back_edge_to_center);
  return std::sqrt((lat_edge_to_center + param.min_turn_radius) *
                       (lat_edge_to_center + param.min_turn_radius) +
                   lon_edge_to_center * lon_edge_to_center);
}

}  // namespace forproto
}  // namespace hqplanner
