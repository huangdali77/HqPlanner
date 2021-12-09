#ifndef HQPLANNER_FOR_PROTO_CONFIG_PARAM_H_
#define HQPLANNER_FOR_PROTO_CONFIG_PARAM_H_

#include <string>

namespace hqplanner {
namespace forproto {
struct ConfigParam {
  /* data */
  const double FLAGS_st_max_s = 40;
  const double FLAGS_trajectory_time_min_interval = 0.02;  // second
  const double FLAGS_trajectory_time_max_interval = 0.1;
  const double FLAGS_trajectory_time_high_density_period = 1.0;
};

}  // namespace forproto
}  // namespace hqplanner

#endif