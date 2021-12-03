#ifndef HQPLANNER_MATH_AABOX2D_H_
#define HQPLANNER_MATH_AABOX2D_H_

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "math_utils.h"
#include "vec2d.h"

namespace hqplanner {
namespace math {
/**
 * @class AABox2d
 * @brief Implements a class of (undirected) axes-aligned bounding boxes in 2-D.
 * This class is referential-agnostic.
 */
class AABox2d {
 public:
  /**
   * @brief Default constructor.
   * Creates an axes-aligned box with zero length and width at the origin.
   */
  AABox2d() = default;
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box with given center, length, and width.
   * @param center The center of the box
   * @param length The size of the box along the x-axis
   * @param width The size of the box along the y-axis
   */
  AABox2d(const Vec2d &center, const double length, const double width);
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box from two opposite corners.
   * @param one_corner One corner of the box
   * @param opposite_corner The opposite corner to the first one
   */
  AABox2d(const Vec2d &one_corner, const Vec2d &opposite_corner);
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box containing all points in a given vector.
   * @param points Vector of points to be included inside the box.
   */
  explicit AABox2d(const std::vector<Vec2d> &points);

  /**
   * @brief Getter of center_
   * @return Center of the box
   */
  const Vec2d &center() const { return center_; }

  /**
   * @brief Getter of x-component of center_
   * @return x-component of the center of the box
   */
  double center_x() const { return center_.x(); }

  /**
   * @brief Getter of y-component of center_
   * @return y-component of the center of the box
   */
  double center_y() const { return center_.y(); }

  /**
   * @brief Getter of length_
   * @return The length of the box
   */
  double length() const { return length_; }

  /**
   * @brief Getter of width_
   * @return The width of the box
   */
  double width() const { return width_; }

  /**
   * @brief Getter of half_length_
   * @return Half of the length of the box
   */
  double half_length() const { return half_length_; }

  /**
   * @brief Getter of half_width_
   * @return Half of the width of the box
   */
  double half_width() const { return half_width_; }

  /**
   * @brief Getter of length_*width_
   * @return The area of the box
   */
  double area() const { return length_ * width_; }

  /**
   * @brief Returns the minimum x-coordinate of the box
   *
   * @return x-coordinate
   */
  double min_x() const { return center_.x() - half_length_; }

  /**
   * @brief Returns the maximum x-coordinate of the box
   *
   * @return x-coordinate
   */
  double max_x() const { return center_.x() + half_length_; }

  /**
   * @brief Returns the minimum y-coordinate of the box
   *
   * @return y-coordinate
   */
  double min_y() const { return center_.y() - half_width_; }

  /**
   * @brief Returns the maximum y-coordinate of the box
   *
   * @return y-coordinate
   */
  double max_y() const { return center_.y() + half_width_; }

  /**
   * @brief Gets all corners in counter clockwise order.
   *
   * @param corners Output where the corners are written
   */
  void GetAllCorners(std::vector<Vec2d> *const corners) const;

  /**
   * @brief Determines whether a given point is in the box.
   *
   * @param point The point we wish to test for containment in the box
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Determines whether a given point is on the boundary of the box.
   *
   * @param point The point we wish to test for boundary membership
   */
  bool IsPointOnBoundary(const Vec2d &point) const;

  /**
   * @brief Determines the distance between a point and the box.
   *
   * @param point The point whose distance to the box we wish to determine.
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Determines the distance between two boxes.
   *
   * @param box Another box.
   */
  double DistanceTo(const AABox2d &box) const;

  /**
   * @brief Determines whether two boxes overlap.
   *
   * @param box Another box
   */
  bool HasOverlap(const AABox2d &box) const;

  /**
   * @brief Shift the center of AABox by the input vector.
   *
   * @param shift_vec The vector by which we wish to shift the box
   */
  void Shift(const Vec2d &shift_vec);

  /**
   * @brief Changes box to include another given box, as well as the current
   * one.
   *
   * @param other_box Another box
   */
  void MergeFrom(const AABox2d &other_box);

  /**
   * @brief Changes box to include a given point, as well as the current box.
   *
   * @param other_point Another point
   */
  void MergeFrom(const Vec2d &other_point);

  /**
   * @brief Gets a human-readable debug string
   *
   * @return A string
   */
  std::string DebugString() const;

 private:
  Vec2d center_;
  double length_ = 0.0;
  double width_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
};

// =====================================函数实现====================================
AABox2d::AABox2d(const Vec2d &center, const double length, const double width)
    : center_(center),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0) {
  if (length_ < -kMathEpsilon || width_ < -kMathEpsilon) {
    assert(0);
  }
  //   CHECK_GT(length_, -kMathEpsilon);
  //   CHECK_GT(width_, -kMathEpsilon);
}

AABox2d::AABox2d(const Vec2d &one_corner, const Vec2d &opposite_corner)
    : AABox2d((one_corner + opposite_corner) / 2.0,
              std::abs(one_corner.x() - opposite_corner.x()),
              std::abs(one_corner.y() - opposite_corner.y())) {}

AABox2d::AABox2d(const std::vector<Vec2d> &points) {
  if (points.empty()) {
    assert(0);
  }
  //   CHECK(!points.empty());
  double min_x = points[0].x();
  double max_x = points[0].x();
  double min_y = points[0].y();
  double max_y = points[0].y();
  for (const auto &point : points) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }

  center_ = {(min_x + max_x) / 2.0, (min_y + max_y) / 2.0};
  length_ = max_x - min_x;
  width_ = max_y - min_y;
  half_length_ = length_ / 2.0;
  half_width_ = width_ / 2.0;
}

void AABox2d::GetAllCorners(std::vector<Vec2d> *const corners) const {
  //   CHECK_NOTNULL(corners)->clear();
  corners->clear();
  corners->reserve(4);
  corners->emplace_back(center_.x() + half_length_, center_.y() - half_width_);
  corners->emplace_back(center_.x() + half_length_, center_.y() + half_width_);
  corners->emplace_back(center_.x() - half_length_, center_.y() + half_width_);
  corners->emplace_back(center_.x() - half_length_, center_.y() - half_width_);
}

bool AABox2d::IsPointIn(const Vec2d &point) const {
  return std::abs(point.x() - center_.x()) <= half_length_ + kMathEpsilon &&
         std::abs(point.y() - center_.y()) <= half_width_ + kMathEpsilon;
}

bool AABox2d::IsPointOnBoundary(const Vec2d &point) const {
  const double dx = std::abs(point.x() - center_.x());
  const double dy = std::abs(point.y() - center_.y());
  return (std::abs(dx - half_length_) <= kMathEpsilon &&
          dy <= half_width_ + kMathEpsilon) ||
         (std::abs(dy - half_width_) <= kMathEpsilon &&
          dx <= half_length_ + kMathEpsilon);
}

double AABox2d::DistanceTo(const Vec2d &point) const {
  const double dx = std::abs(point.x() - center_.x()) - half_length_;
  const double dy = std::abs(point.y() - center_.y()) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

double AABox2d::DistanceTo(const AABox2d &box) const {
  const double dx =
      std::abs(box.center_x() - center_.x()) - box.half_length() - half_length_;
  const double dy =
      std::abs(box.center_y() - center_.y()) - box.half_width() - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

bool AABox2d::HasOverlap(const AABox2d &box) const {
  return std::abs(box.center_x() - center_.x()) <=
             box.half_length() + half_length_ &&
         std::abs(box.center_y() - center_.y()) <=
             box.half_width() + half_width_;
}

void AABox2d::Shift(const Vec2d &shift_vec) { center_ += shift_vec; }

void AABox2d::MergeFrom(const AABox2d &other_box) {
  const double x1 = std::min(min_x(), other_box.min_x());
  const double x2 = std::max(max_x(), other_box.max_x());
  const double y1 = std::min(min_y(), other_box.min_y());
  const double y2 = std::max(max_y(), other_box.max_y());
  center_ = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0);
  length_ = x2 - x1;
  width_ = y2 - y1;
  half_length_ = length_ / 2.0;
  half_width_ = width_ / 2.0;
}

void AABox2d::MergeFrom(const Vec2d &other_point) {
  const double x1 = std::min(min_x(), other_point.x());
  const double x2 = std::max(max_x(), other_point.x());
  const double y1 = std::min(min_y(), other_point.y());
  const double y2 = std::max(max_y(), other_point.y());
  center_ = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0);
  length_ = x2 - x1;
  width_ = y2 - y1;
  half_length_ = length_ / 2.0;
  half_width_ = width_ / 2.0;
}

// std::string AABox2d::DebugString() const {
//   return util::StrCat("aabox2d ( center = ", center_.DebugString(),
//                       "  length = ", length_, "  width = ", width_, " )");
// }
}  // namespace math
}  // namespace hqplanner

#endif /* MODULES_COMMON_MATH_AABOX2D_H_ */
