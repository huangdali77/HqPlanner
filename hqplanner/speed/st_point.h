#ifndef MODULES_PLANNING_COMMON_SPEED_ST_POINT_H_
#define MODULES_PLANNING_COMMON_SPEED_ST_POINT_H_

#include <iomanip>
#include <string>

#include "math/vec2d.h"
namespace hqplanner {
namespace speed {

class STPoint : public hqplanner::math::Vec2d {
 public:
  STPoint() = default;
  STPoint(const double s, const double t);
  explicit STPoint(const hqplanner::math::Vec2d& vec2d_point);

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);
  std::string DebugString() const;
};

// ===================函数实现======================
STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const hqplanner::math::Vec2d& vec2d_point)
    : Vec2d(vec2d_point) {}

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { return set_y(s); }

void STPoint::set_t(const double t) { return set_x(t); }
}  // namespace speed
}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_SPEED_ST_POINT_H_
