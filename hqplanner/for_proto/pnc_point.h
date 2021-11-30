#include <string>
struct PathPoint {
  // coordinates
  double x;
  double y;
  double z;

  // direction on the x-y plane
  double theta;
  // curvature on the x-y planning
  double kappa;
  // accumulated distance from beginning of the path
  double s;

  // derivative of kappa w.r.t s.
  double dkappa;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa;
  // The lane ID where the path point is on
  std::string lane_id;
};

struct TrajectoryPoint {
  // path point
  PathPoint path_point;

  // linear velocity
  double v;  // in [m/s]
  double v_x;
  double v_y;
  // linear acceleration
  double a;
  double a_x;
  double a_y;
  // relative time from beginning of the trajectory
  double relative_time;
};

struct AnchorPoint {
 public:
  AnchorPoint() = default;
  AnchorPoint(double x, double y, double s)
      : cartesian_x(x), cartesian_y(y), frenet_s(s) {}
  // x-y plane
  double cartesian_x;
  double cartesian_y;
  // frenet plane
  double frenet_s;
  /* data */
};

struct ReferencePoint {
 public:
  ReferencePoint() = default;
  ReferencePoint(double xx, double yy, double ss) : x(xx), y(yy), s(ss) {}
  double x;
  double y;
  double s;
  double yaw;
  double curvature;
  double d_curvature;
};