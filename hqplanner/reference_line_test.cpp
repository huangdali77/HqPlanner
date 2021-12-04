#include "reference_line.h"

#include <graphics.h>

#include <cmath>
#include <iostream>
#include <vector>
using hqplanner::ReferenceLine;
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::ReferencePoint;
int main() {
  std::vector<double> x = {
      92.2679315918, 91.7368744032, 91.2058172146, 90.6747600259, 90.1217327927,
      89.3523723898, 88.4440988632, 87.4743322588, 86.5204926224, 85.66,
      84.8945767188, 84.1008630548, 83.2800137176, 82.4331834167, 81.5615268616,
      80.6661987617, 79.7483538266, 78.8091467658, 77.8497322887, 76.871265105,
      75.874899924,  74.8617914552, 73.8330944082, 72.7899634925, 71.7335534176,
      70.6650188929, 69.585514628,  68.4961953323, 67.3982157154, 66.2927304867,
      65.1808943558, 64.0638620322, 62.9427882253, 61.8188276447, 60.6931349998,
      59.5668650002, 58.4411723553, 57.3172117747, 56.1961379678, 55.0791056442,
      53.9672695133, 52.8617842846, 51.7638046677, 50.674485372,  49.5949811071,
      48.5264465824, 47.4700365075, 46.4269055918, 45.3982085448, 44.385100076,
      43.388734895,  42.4102677113, 41.4508532342, 40.5116461734, 39.5938012383,
      38.6984731384, 37.8268165833, 36.9799862824, 36.1591369452, 35.3654232812,
      34.6};
  std::vector<double> y = {
      -100.213542235, -101.060878215, -101.908214194, -102.755550174,
      -103.50964406,  -104.362357115, -105.205219287, -105.929760526,
      -106.42751078,  -106.59,        -106.549869862, -106.508225929,
      -106.465129988, -106.420643827, -106.374829234, -106.327747996,
      -106.2794619,   -106.230032734, -106.179522286, -106.127992344,
      -106.075504694, -106.022121124, -105.967903423, -105.912913377,
      -105.857212774, -105.800863401, -105.743927047, -105.686465499,
      -105.628540543, -105.570213969, -105.511547563, -105.452603113,
      -105.393442407, -105.334127232, -105.274719375, -105.215280625,
      -105.155872768, -105.096557593, -105.037396887, -104.978452437,
      -104.919786031, -104.861459457, -104.803534501, -104.746072953,
      -104.689136599, -104.632787226, -104.577086623, -104.522096577,
      -104.467878876, -104.414495306, -104.362007656, -104.310477714,
      -104.259967266, -104.2105381,   -104.162252004, -104.115170766,
      -104.069356173, -104.024870012, -103.981774071, -103.940130138,
      -103.9};
  ReferenceLine ref_line(x, y);
  std::vector<ReferencePoint> reference_line_points =
      ref_line.GetReferenceLinePoints();

  std::cout << reference_line_points.size() << std::endl;
  std::vector<AnchorPoint> anchor_points = ref_line.GetAnchorPoints();
  // --------------------------------------------------------------------------------------------------
  // x=[0,640],y=[0,480]

  int gd = DETECT, gm = 0;
  initgraph(&gd, &gm, "");
  // anchor point
  //   double m = 10, c = -800, b = -300;
  //   for (int i = 0; i < anchor_points.size() - 1; ++i) {
  //     line(m * anchor_points[i].cartesian_x + b,
  //          m * abs(anchor_points[i].cartesian_y) + c,
  //          m * anchor_points[i + 1].cartesian_x + b,
  //          m * abs(anchor_points[i + 1].cartesian_y) + c);
  //   }

  //   位置点
  // double m = 10, c = -800, b = -350;
  // for (int i = 0; i < reference_line_points.size() - 1; ++i) {
  //   line(m * reference_line_points[i].x + b,
  //        m * abs(reference_line_points[i].y) + c,
  //        m * reference_line_points[i + 1].x + b,
  //        m * abs(reference_line_points[i + 1].y) + c);
  // }

  //   偏航角
  double m = 5, c = 300, b = 100;
  for (int i = 0; i < reference_line_points.size() - 1; ++i) {
    line(m * reference_line_points[i].s + b,
         m * (reference_line_points[i].yaw) + c,
         m * reference_line_points[i + 1].s + b,
         m * (reference_line_points[i + 1].yaw) + c);
  }
  line(0, c, 640, c);

  // double m = 5, c = 200, b = 200;
  // for (int i = 0; i < reference_line_points.size() - 1; ++i) {
  //   line(m * reference_line_points[i].s + b,
  //        m * (reference_line_points[i].curvature) + c,
  //        m * reference_line_points[i + 1].s + b,
  //        m * (reference_line_points[i + 1].curvature) + c);
  // }
  for (int i = 0; i < reference_line_points.size(); ++i) {
    std::cout << 1 / reference_line_points[i].curvature << std::endl;
  }
  delay(50000);
  closegraph();
  return 0;
}
