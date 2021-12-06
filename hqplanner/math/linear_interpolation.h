/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief Linear interpolation functions.
 */

#ifndef MODULES_COMMON_MATH_LINEAR_INTERPOLATION_H_
#define MODULES_COMMON_MATH_LINEAR_INTERPOLATION_H_

#include <assert.h>

#include <cmath>

#include "for_proto/pnc_point.h"
#include "math/math_utils.h"
/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace hqplanner {
namespace math {

using hqplanner::forproto::PathPoint;
using hqplanner::forproto::SLPoint;
using hqplanner::forproto::TrajectoryPoint;

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

/**
 * @brief Spherical linear interpolation between two angles.
 *        The two angles are within range [-M_PI, M_PI).
 * @param a0 The value of the first angle.
 * @param t0 The interpolation parameter of the first angle.
 * @param a1 The value of the second angle.
 * @param t1 The interpolation parameter of the second angle.
 * @param t The interpolation parameter for interpolation.
 * @param a The value of the spherically interpolated angle.
 * @return Interpolated angle.
 */
// double slerp(const double a0, const double t0, const double a1, const double
// t1,
//              const double t);

// SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
//                                             const SLPoint &p1, const double
//                                             w);

// PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
//                                               const PathPoint &p1,
//                                               const double s);

// TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint
// &tp0,
//                                                     const TrajectoryPoint
//                                                     &tp1, const double t);

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w) {
  //   CHECK_GE(w, 0.0);
  assert(w >= 0);
  SLPoint p;
  p.s = (1 - w) * p0.s + w * p1.s;

  p.l = (1 - w) * p0.l + w * p1.l;
  return p;
}

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s) {
  double s0 = p0.s;
  double s1 = p1.s;
  assert(s0 <= s1);
  //   CHECK_LE(s0, s1);

  PathPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x + weight * p1.x;
  double y = (1 - weight) * p0.y + weight * p1.y;
  double theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  double kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  double dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
  double ddkappa = (1 - weight) * p0.ddkappa + weight * p1.ddkappa;
  path_point.x = x;
  path_point.y = y;
  path_point.theta = theta;
  path_point.kappa = kappa;
  path_point.dkappa = dkappa;
  path_point.ddkappa = ddkappa;
  path_point.s = s;
  return path_point;
}

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0,
                                                    const TrajectoryPoint &tp1,
                                                    const double t) {
  //   if (!tp0.has_path_point() || !tp1.has_path_point()) {
  //     TrajectoryPoint p;
  //     p.mutable_path_point()->CopyFrom(PathPoint());
  //     return p;
  //   }
  const PathPoint pp0 = tp0.path_point;
  const PathPoint pp1 = tp1.path_point;
  double t0 = tp0.relative_time;
  double t1 = tp1.relative_time;

  TrajectoryPoint tp;
  tp.v = lerp(tp0.v, t0, tp1.v, t1, t);
  tp.a = lerp(tp0.a, t0, tp1.a, t1, t);
  tp.relative_time = t;

  PathPoint path_point;
  //   PathPoint *path_point = tp.mutable_path_point();
  path_point.x = lerp(pp0.x, t0, pp1.x, t1, t);
  path_point.y = lerp(pp0.y, t0, pp1.y, t1, t);
  path_point.theta = slerp(pp0.theta, t0, pp1.theta, t1, t);
  path_point.kappa = lerp(pp0.kappa, t0, pp1.kappa, t1, t);
  path_point.dkappa = lerp(pp0.dkappa, t0, pp1.dkappa, t1, t);
  path_point.ddkappa = lerp(pp0.ddkappa, t0, pp1.ddkappa, t1, t);
  path_point.s = lerp(pp0.s, t0, pp1.s, t1, t);
  tp.path_point = path_point;
  return tp;
}
}  // namespace math

}  // namespace hqplanner

#endif  // MODULES_COMMON_MATH_LINEAR_INTERPOLATION_H_
