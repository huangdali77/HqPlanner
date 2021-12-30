// #include "hqplanner/cubic_spline.h"

// CubicSpline::CubicSpline(std::vector<double>& x, std::vector<double>& y)
//     : spline_anchor_points_x_(x), spline_anchor_points_y_(y) {
//   anchor_points_num_ = spline_anchor_points_x_.size();
//   CalculateSplineCoefs();
// }

// void CubicSpline::CalculateSplineCoefs() {
//   std::vector<double> h(anchor_points_num_ - 1, 0);
//   for (int i = 0; i < anchor_points_num_ - 1; ++i) {
//     h[i] = spline_anchor_points_x_[i + 1] - spline_anchor_points_x_[i];
//   }
//   spline_coefs_a_ = spline_anchor_points_y_;

//   Matrix<double, Dynamic, Dynamic> A;
//   Matrix<double, Dynamic, Dynamic> b;
//   A = CalculateAMtrix(h);
//   b = CalculateBMtrix(h);
//   VectorXd spline_coefs_c_vectorxd = A.colPivHouseholderQr().solve(b);
//   for (int i = 0; i < spline_coefs_c_vectorxd.rows(); ++i) {
//     spline_coefs_c_.push_back(spline_coefs_c_vectorxd(i));
//   }

//   for (int i = 0; i < anchor_points_num_ - 1; ++i) {
//     double td = (spline_coefs_c_[i + 1] - spline_coefs_c_[i]) / (3.0 * h[i]);
//     spline_coefs_d_.push_back(td);
//     double tb =
//         (spline_coefs_a_[i + 1] - spline_coefs_a_[i]) / h[i] -
//         h[i] * (spline_coefs_c_[i + 1] + 2.0 * spline_coefs_c_[i]) / 3.0;
//     spline_coefs_b_.push_back(tb);
//   }
// }

// MatrixXd CubicSpline::CalculateAMtrix(const std::vector<double>& h) {
//   MatrixXd A = MatrixXd::Zero(anchor_points_num_, anchor_points_num_);
//   A(0, 0) = 1.0;
//   for (int i = 0; i < anchor_points_num_ - 1; ++i) {
//     if (i != anchor_points_num_ - 2) {
//       A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);
//     }
//     A(i + 1, i) = h[i];
//     A(i, i + 1) = h[i];
//   }

//   A(0, 1) = 0.0;
//   A(anchor_points_num_ - 1, anchor_points_num_ - 2) = 0.0;
//   A(anchor_points_num_ - 1, anchor_points_num_ - 1) = 1.0;

//   return A;
// }

// MatrixXd CubicSpline::CalculateBMtrix(const std::vector<double>& h) {
//   MatrixXd b = MatrixXd::Zero(anchor_points_num_, 1);
//   for (int i = 0; i < anchor_points_num_ - 2; ++i) {
//     b(i + 1, 0) =
//         3.0 * (spline_coefs_a_[i + 2] - spline_coefs_a_[i + 1]) / h[i + 1] -
//         3.0 * (spline_coefs_a_[i + 1] - spline_coefs_a_[i]) / h[i];
//   }
//   return b;
// }

// bool CubicSpline::GetSplinePointValue(double t, double* result) {
//   if (t < spline_anchor_points_x_.front() ||
//       t > spline_anchor_points_x_.back()) {
//     return false;
//   }

//   std::vector<double>::iterator index_it = std::upper_bound(
//       spline_anchor_points_x_.begin(), spline_anchor_points_x_.end(), t);
//   int index_i = int(index_it - spline_anchor_points_x_.begin()) - 1;

//   double dx = t - spline_anchor_points_x_[index_i];
//   *result = spline_coefs_a_[index_i] + spline_coefs_b_[index_i] * dx +
//             spline_coefs_c_[index_i] * dx * dx +
//             spline_coefs_d_[index_i] * dx * dx * dx;
//   return true;
// }

// bool CubicSpline::GetSplinePointFirstDerivativeValue(double t, double*
// result) {
//   if (t < spline_anchor_points_x_.front() ||
//       t > spline_anchor_points_x_.back()) {
//     return false;
//   }

//   std::vector<double>::iterator index_it = std::upper_bound(
//       spline_anchor_points_x_.begin(), spline_anchor_points_x_.end(), t);

//   int index_i = int(index_it - spline_anchor_points_x_.begin()) - 1;

//   double dx = t - spline_anchor_points_x_[index_i];
//   *result = spline_coefs_b_[index_i] + 2.0 * spline_coefs_c_[index_i] * dx +
//             3.0 * spline_coefs_d_[index_i] * dx * dx;
//   return true;
// }

// bool CubicSpline::GetSplinePointSecondDerivativeValue(double t,
//                                                       double* result) {
//   if (t < spline_anchor_points_x_.front() ||
//       t > spline_anchor_points_x_.back()) {
//     return false;
//   }

//   std::vector<double>::iterator index_it = std::upper_bound(
//       spline_anchor_points_x_.begin(), spline_anchor_points_x_.end(), t);

//   int index_i = int(index_it - spline_anchor_points_x_.begin()) - 1;

//   double dx = t - spline_anchor_points_x_[index_i];

//   *result =
//       2.0 * spline_coefs_c_[index_i] + 6.0 * spline_coefs_d_[index_i] * dx;
//   return true;
// }