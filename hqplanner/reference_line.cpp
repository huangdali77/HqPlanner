
// #include "hqplanner/reference_line.h"
#include "reference_line.h"

#include <cmath>
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
  ConstructReferenceLine();
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
void ReferenceLine::ConstructReferenceLine() {
  anchor_points_x_s_ = CubicSpline(anchor_points_s_, anchor_points_x_);
  anchor_points_y_s_ = CubicSpline(anchor_points_s_, anchor_points_y_);
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
    double i_x;
    anchor_points_x_s_.GetSplinePointValue(i_s, &i_x);
    reference_line_points_.back().x = i_x;
    // y
    double i_y;
    anchor_points_y_s_.GetSplinePointValue(i_s, &i_y);
    reference_line_points_.back().y = i_y;
    // yaw
    double i_dx, i_dy;
    anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s, &i_dx);
    anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s, &i_dy);
    reference_line_points_.back().yaw = atan2(i_dy, i_dx);
    // curvature
    double i_ddx, i_ddy;
    anchor_points_x_s_.GetSplinePointSecondDerivativeValue(i_s, &i_ddx);
    anchor_points_y_s_.GetSplinePointSecondDerivativeValue(i_s, &i_ddy);
    reference_line_points_.back().curvature =
        (i_ddy * i_dx - i_ddx * i_dy) / (i_dx * i_dx + i_dy * i_dy);
  }
  for (int i = 0; i < reference_line_points_.size() - 1; ++i) {
    reference_line_points_[i].d_curvature =
        (reference_line_points_[i + 1].curvature -
         reference_line_points_[i].curvature) /
        REFERENCE_LINE_SAMPLE_STEP;
  }
  reference_line_points_.back().d_curvature =
      reference_line_points_[reference_line_points_.size() - 2].d_curvature;
}

// =======================备用方案=============================================
void ReferenceLine::ConstructReferenceLine2() {
  anchor_points_x_s_ = CubicSpline(anchor_points_s_, anchor_points_x_);
  anchor_points_y_s_ = CubicSpline(anchor_points_s_, anchor_points_y_);
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
    double i_x;
    anchor_points_x_s_.GetSplinePointValue(i_s, &i_x);
    reference_line_points_.back().x = i_x;
    // y
    double i_y;
    anchor_points_y_s_.GetSplinePointValue(i_s, &i_y);
    reference_line_points_.back().y = i_y;
    // ==============================================================
    // yaw
    // double i_dx, i_dy;
    // anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s, &i_dx);
    // anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s, &i_dy);
    // reference_line_points_.back().yaw = atan2(i_dy, i_dx);
    // curvature
    // double i_ddx, i_ddy;
    // anchor_points_x_s_.GetSplinePointSecondDerivativeValue(i_s, &i_ddx);
    // anchor_points_y_s_.GetSplinePointSecondDerivativeValue(i_s, &i_ddy);
    // reference_line_points_.back().curvature =
    //     (i_ddy * i_dx - i_ddx * i_dy) / (i_dx * i_dx + i_dy * i_dy);
  }
  // yaw
  for (int i = 0; i < reference_line_points_.size() - 1; ++i) {
    double dx = reference_line_points_[i + 1].x - reference_line_points_[i].x;
    double dy = reference_line_points_[i + 1].y - reference_line_points_[i].y;
    reference_line_points_[i].yaw = atan2(dy, dx);
  }
  reference_line_points_.back().yaw =
      reference_line_points_[reference_line_points_.size() - 2].yaw;

  for (int i = 0; i < reference_line_points_.size() - 1; ++i) {
    reference_line_points_[i].d_curvature =
        (reference_line_points_[i + 1].curvature -
         reference_line_points_[i].curvature) /
        REFERENCE_LINE_SAMPLE_STEP;
  }
  reference_line_points_.back().d_curvature =
      reference_line_points_[reference_line_points_.size() - 2].d_curvature;
}