#ifndef HQPLANNER_REFERENCE_LINE_H_
#define HQPLANNER_REFERENCE_LINE_H_
#include <cmath>
#include <utility>
#include <vector>
// #include "hqplanner/cubic_spline.h"

// #include "hqplanner/cubic_spline_2d.h"
// #include "hqplanner/for_proto/pnc_point.h"
#include "for_proto/pnc_point.h"
#include "math/cubic_spline.h"

namespace hqplanner {
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::ReferencePoint;
using hqplanner::forproto::SLPoint;
using hqplanner::math::CubicSpline;
using hqplanner::math::Vec2d;
double REFERENCE_LINE_SAMPLE_STEP = 0.1;

class ReferenceLine {
 public:
  ReferenceLine() = default;
  explicit ReferenceLine(const std::vector<double>& x,
                         const std::vector<double>& y);

  void AccumulateOnS();
  void ConstructReferenceLineByFixedStep();
  void ConstructReferenceLine2();
  double ComputeCurvature(double dx, double ddx, double dy, double ddy);
  double ComputeCurvatureDerivative(double dx, double ddx, double dddx,
                                    double dy, double ddy, double dddy);
  std::vector<ReferencePoint> GetReferenceLinePoints() {
    return reference_line_points_;
  }

  double Length() const { return reference_line_points_.back().s; }

  double GetPositionXByS(double i_s);
  double GetPositionYByS(double i_s);
  double GetHeadingByS(double i_s);
  double GetKappaByS(double i_s);
  double GetKappaDerivativeByS(double i_s);
  // ReferencePoint GetReferencePoint(const double s) const;
  std::vector<AnchorPoint> GetAnchorPoints() { return anchor_points_; }

  bool SLToXY(const SLPoint& sl_point, Vec2d* const xy_point);
  bool XYToSL(const math::Vec2d& xy_point, SLPoint* const sl_point);

  ReferencePoint GetReferencePoint(const double x, const double y) const;
  ReferencePoint GetReferencePoint(const double s);

 private:
  std::vector<AnchorPoint> anchor_points_;
  std::vector<double> anchor_points_x_;
  std::vector<double> anchor_points_y_;
  std::vector<double> anchor_points_s_;
  CubicSpline anchor_points_x_s_;
  CubicSpline anchor_points_y_s_;
  std::vector<ReferencePoint> reference_line_points_;
};

// =========================函数实现==================================================================

ReferenceLine::ReferenceLine(const std::vector<double>& x,
                             const std::vector<double>& y)
    : anchor_points_x_(x), anchor_points_y_(y) {
  std::vector<AnchorPoint> anchor_points(x.size());

  for (int i = 0; i < x.size(); ++i) {
    anchor_points[i].cartesian_x = x[i];
    anchor_points[i].cartesian_y = y[i];
  }
  anchor_points_ = anchor_points;

  AccumulateOnS();
  anchor_points_x_s_ = CubicSpline(anchor_points_s_, anchor_points_x_);
  anchor_points_y_s_ = CubicSpline(anchor_points_s_, anchor_points_y_);
  ConstructReferenceLineByFixedStep();  //供寻找最近点使用
}

void ReferenceLine::AccumulateOnS() {
  anchor_points_[0].frenet_s = 0;
  anchor_points_s_.resize(anchor_points_.size());
  anchor_points_s_[0] = 0;

  for (int i = 1; i < anchor_points_.size(); ++i) {
    double ds = hypot(
        anchor_points_[i].cartesian_x - anchor_points_[i - 1].cartesian_x,
        anchor_points_[i].cartesian_y - anchor_points_[i - 1].cartesian_y);
    anchor_points_[i].frenet_s = anchor_points_[i - 1].frenet_s + ds;
    anchor_points_s_[i] = anchor_points_[i].frenet_s;
  }
}

double ReferenceLine::ComputeCurvature(double dx, double ddx, double dy,
                                       double ddy) {
  double a = dx * ddy - dy * ddx;
  double norm_square = dx * dx + dy * dy;
  double norm = sqrt(norm_square);
  double b = norm * norm_square;
  return a / b;
}

double ReferenceLine::ComputeCurvatureDerivative(double dx, double ddx,
                                                 double dddx, double dy,
                                                 double ddy, double dddy) {
  double a = dx * ddy - dy * ddx;
  double b = dx * dddy - dy * dddx;
  double c = dx * ddx + dy * ddy;
  double d = dx * dx + dy * dy;
  return (b * d - 3.0 * a * c) / (d * d * d);
}

bool ReferenceLine::SLToXY(const SLPoint& sl_point, Vec2d* const xy_point) {
  assert(xy_point != nullptr);

  if (reference_line_points_.size() < 2) {
    return false;
  }

  const auto angle = GetHeadingByS(sl_point.s);
  const auto x = GetPositionXByS(sl_point.s);
  const auto y = GetPositionYByS(sl_point.s);

  xy_point->set_x(x - std::sin(angle) * sl_point.l);
  xy_point->set_y(y + std::cos(angle) * sl_point.l);
  return true;
}

bool ReferenceLine::XYToSL(const Vec2d& xy_point, SLPoint* const sl_point) {
  assert(sl_point != nullptr);
  // DCHECK_NOTNULL(sl_point);
  double s = 0.0;
  double l = 0.0;

  if (!map_path_.GetProjection(xy_point, &s, &l)) {
    AERROR << "Can't get nearest point from path.";
    return false;
  }
  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

// ReferencePoint ReferenceLine::GetReferencePoint(const double s) const {
//   if (s<reference_line_points_.front().s-1e-2){
//     return reference_line_points_.front();
//   }
//     if (s > reference_line_points_.back().s + 1e-2) {
//     return reference_line_points_.back();
//   }

//   auto interpolate_index = map_path_.GetIndexFromS(s);

//   uint32_t index = interpolate_index.id;
//   uint32_t next_index = index + 1;
//   if (next_index >= reference_points_.size()) {
//     next_index = reference_points_.size() - 1;
//   }

//   const auto& p0 = reference_points_[index];
//   const auto& p1 = reference_points_[next_index];

//   const double s0 = accumulated_s[index];
//   const double s1 = accumulated_s[next_index];
//   return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
// }

double ReferenceLine::GetPositionXByS(double i_s) {
  return anchor_points_x_s_.GetSplinePointValue(i_s);
}

double ReferenceLine::GetPositionYByS(double i_s) {
  return anchor_points_y_s_.GetSplinePointValue(i_s);
}

double ReferenceLine::GetHeadingByS(double i_s) {
  double i_dx = anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s);
  double i_dy = anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s);
  return atan2(i_dy, i_dx);
}

double ReferenceLine::GetKappaByS(double i_s) {
  double i_dx = anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s);
  double i_dy = anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s);
  double i_ddx = anchor_points_x_s_.GetSplinePointSecondDerivativeValue(i_s);
  double i_ddy = anchor_points_y_s_.GetSplinePointSecondDerivativeValue(i_s);
  return ComputeCurvature(i_dx, i_ddx, i_dy, i_ddy);
}

double ReferenceLine::GetKappaDerivativeByS(double i_s) {
  double i_dx = anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s);
  double i_dy = anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s);
  double i_ddx = anchor_points_x_s_.GetSplinePointSecondDerivativeValue(i_s);
  double i_ddy = anchor_points_y_s_.GetSplinePointSecondDerivativeValue(i_s);
  double i_dddx = anchor_points_x_s_.GetSplinePointThirdDerivativeValue(i_s);
  double i_dddy = anchor_points_y_s_.GetSplinePointThirdDerivativeValue(i_s);
  return ComputeCurvatureDerivative(i_dx, i_ddx, i_dddx, i_dy, i_ddy, i_dddy);
}

ReferencePoint ReferenceLine::GetReferencePoint(const double s) {
  ReferencePoint ref_point(s);

  ref_point.x = anchor_points_x_s_.GetSplinePointValue(s);
  ref_point.y = anchor_points_y_s_.GetSplinePointValue(s);

  double dx = anchor_points_x_s_.GetSplinePointFirstDerivativeValue(s);
  double dy = anchor_points_y_s_.GetSplinePointFirstDerivativeValue(s);
  ref_point.heading = atan2(dy, dx);

  double ddx = anchor_points_x_s_.GetSplinePointSecondDerivativeValue(s);
  double ddy = anchor_points_y_s_.GetSplinePointSecondDerivativeValue(s);
  ref_point.kappa = ComputeCurvature(dx, ddx, dy, ddy);

  double dddx = anchor_points_x_s_.GetSplinePointThirdDerivativeValue(s);
  double dddy = anchor_points_y_s_.GetSplinePointThirdDerivativeValue(s);
  ref_point.dkappa = ComputeCurvatureDerivative(dx, ddx, dddx, dy, ddy, dddy);

  return ref_point;
}

// ReferencePoint ReferenceLine::GetReferencePoint(const double x,
//                                                 const double y) const {

//                                                 }

void ReferenceLine::ConstructReferenceLineByFixedStep() {
  int reference_line_points_num =
      int(anchor_points_s_.back() / REFERENCE_LINE_SAMPLE_STEP);
  std::vector<double> ref_s(reference_line_points_num, 0);
  for (int i = 1; i < ref_s.size(); ++i) {
    ref_s[i] = ref_s[i - 1] + REFERENCE_LINE_SAMPLE_STEP;
  }
  for (auto i_s : ref_s) {
    reference_line_points_.emplace_back();
    reference_line_points_.back().s = i_s;
    // x

    double i_x = anchor_points_x_s_.GetSplinePointValue(i_s);
    reference_line_points_.back().x = i_x;
    // y
    double i_y = anchor_points_y_s_.GetSplinePointValue(i_s);
    reference_line_points_.back().y = i_y;

    // yaw
    double i_dx = anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s);
    double i_dy = anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s);
    reference_line_points_.back().heading = atan2(i_dy, i_dx);

    // curvature
    double i_ddx = anchor_points_x_s_.GetSplinePointSecondDerivativeValue(i_s);
    double i_ddy = anchor_points_y_s_.GetSplinePointSecondDerivativeValue(i_s);
    reference_line_points_.back().kappa =
        ComputeCurvature(i_dx, i_ddx, i_dy, i_ddy);

    // d_curvature
    double i_dddx = anchor_points_x_s_.GetSplinePointThirdDerivativeValue(i_s);
    double i_dddy = anchor_points_y_s_.GetSplinePointThirdDerivativeValue(i_s);
    reference_line_points_.back().dkappa =
        ComputeCurvatureDerivative(i_dx, i_ddx, i_dddx, i_dy, i_ddy, i_dddy);
  }
  // for (int i = 0; i < reference_line_points_.size() - 1; ++i) {
  //   reference_line_points_[i].d_curvature =
  //       (reference_line_points_[i + 1].curvature -
  //        reference_line_points_[i].curvature) /
  //       REFERENCE_LINE_SAMPLE_STEP;
  // }
  // reference_line_points_.back().d_curvature =
  //     reference_line_points_[reference_line_points_.size() - 2].d_curvature;
}

// =======================备用方案=============================================
// void ReferenceLine::ConstructReferenceLine2() {
//   anchor_points_x_s_ = CubicSpline(anchor_points_s_, anchor_points_x_);
//   anchor_points_y_s_ = CubicSpline(anchor_points_s_, anchor_points_y_);
//   int reference_line_points_num =
//       int(anchor_points_s_.back() / REFERENCE_LINE_SAMPLE_STEP);
//   std::vector<double> ref_s(reference_line_points_num, 0);
//   for (int i = 1; i < ref_s.size(); ++i) {
//     ref_s[i] = ref_s[i - 1] + REFERENCE_LINE_SAMPLE_STEP;
//   }
//   for (auto i_s : ref_s) {
//     // reference_line_points_.emplace_back();
//     // reference_line_points_.back().s = i_s;
//     // // x
//     // double i_x;
//     // anchor_points_x_s_.GetSplinePointValue(i_s, &i_x);
//     // reference_line_points_.back().x = i_x;
//     // // y
//     // double i_y;
//     // anchor_points_y_s_.GetSplinePointValue(i_s, &i_y);
//     // reference_line_points_.back().y = i_y;
//     // ==============================================================
//     // yaw
//     // double i_dx, i_dy;
//     // anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s, &i_dx);
//     // anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s, &i_dy);
//     // reference_line_points_.back().yaw = atan2(i_dy, i_dx);
//     // curvature
//     // double i_ddx, i_ddy;
//     // anchor_points_x_s_.GetSplinePointSecondDerivativeValue(i_s, &i_ddx);
//     // anchor_points_y_s_.GetSplinePointSecondDerivativeValue(i_s, &i_ddy);
//     // reference_line_points_.back().curvature =
//     //     (i_ddy * i_dx - i_ddx * i_dy) / (i_dx * i_dx + i_dy * i_dy);
//   }
//   // yaw
//   for (int i = 0; i < reference_line_points_.size() - 1; ++i) {
//     double dx = reference_line_points_[i + 1].x -
//     reference_line_points_[i].x; double dy = reference_line_points_[i + 1].y
//     - reference_line_points_[i].y; reference_line_points_[i].yaw = atan2(dy,
//     dx);
//   }
//   reference_line_points_.back().yaw =
//       reference_line_points_[reference_line_points_.size() - 2].yaw;

//   for (int i = 0; i < reference_line_points_.size() - 1; ++i) {
//     reference_line_points_[i].d_curvature =
//         (reference_line_points_[i + 1].curvature -
//          reference_line_points_[i].curvature) /
//         REFERENCE_LINE_SAMPLE_STEP;
//   }
//   reference_line_points_.back().d_curvature =
//       reference_line_points_[reference_line_points_.size() - 2].d_curvature;
// }
}  // namespace hqplanner

#endif