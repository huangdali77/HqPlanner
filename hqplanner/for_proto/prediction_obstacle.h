
#include <vector>

#include "perception_obstacle.h";
#include "pnc_point.h";
namespace hqplanner {
namespace forproto {
struct Trajectory {
  double probability;  // probability of this trajectory
  std::vector<TrajectoryPoint> trajectory_point;
};

struct PredictionObstacle {
  PerceptionObstacle perception_obstacle;
  double timestamp;  // GPS time in seconds
  // the length of the time for this prediction (e.g. 10s)
  double predicted_period;
  // can have multiple trajectories per obstacle
  std::vector<Trajectory> trajectory;
};

struct PredictionObstacles {
  // make prediction for multiple obstacles
  std::vector<PredictionObstacle> prediction_obstacle;
  // start timestamp
  double start_timestamp;
  // end timestamp
  double end_timestamp;
}

}  // namespace forproto
}  // namespace hqplanner
