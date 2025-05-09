/******************************************************************************
* FILENAME:     math.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     03.07.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/utils/math.hpp>

namespace ikf {
namespace utils {

double rad2deg(const double rad) {
  return rad * (180/M_PI);
}

double deg2rad(const double deg) {
  return deg * (M_PI/180);
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d M;
  M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return M;
}

Eigen::Vector3d skew_inv(const Eigen::Matrix3d& M) {
  Eigen::Vector3d v(M(2, 1), M(0, 2), M(1, 0));
  return v;
}

Eigen::Matrix4d OmegaMat(const Eigen::Vector3d& v)
{
  Eigen::Matrix4d res;
  res.setZero();

  res.block(0, 1, 1, 3) = -v.transpose();
  res.block(1, 0, 3, 1) = v;
  res.block(1, 1, 3, 3) = -skew(v);

  return res;
}

Eigen::Matrix4d MatExp(const Eigen::Matrix4d& A, const int order)
{
  Eigen::Matrix4d matexp(Eigen::Matrix4d::Identity());  // initial condition with k=0

  int div = 1;
  Eigen::Matrix4d a_loop = A;

  for (int k = 1; k <= order; k++)
  {
    div = div * k;  // factorial(k)
    matexp = matexp + a_loop / div;
    a_loop = a_loop * A;  // adding one exponent each iteration
  }

  return matexp;
}

double wrapMax(const double x, const double max) {
  /* integer math: `(max + x % max) % max` */
  return fmod(max + fmod(x, max), max);
}

double wrapMinMax(const double x, const double min, const double max) { return min + wrapMax(x - min, max - min); }

double wrapToPi(const double x_rad) { return wrapMinMax(x_rad, -M_PI, +M_PI); }

double wrapTo2Pi(const double x_rad) { return wrapMinMax(x_rad, -M_2_PI, +M_2_PI); }

double wrapTo180deg(const double x_deg) { return wrapMinMax(x_deg, -180, +180); }

double wrapTo360deg(const double x_deg) { return wrapMinMax(x_deg, -360, +360); }

void quat2rpy(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
  roll = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  pitch = std::asin(q.w() * q.y() - q.z() * q.x());
  yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

void quat2roll(const Eigen::Quaterniond &q, double &roll) {
  roll = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
}

void quat2pitch(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
  pitch = std::asin(q.w() * q.y() - q.z() * q.x());
}

void quat2yaw(const Eigen::Quaterniond &q, double &yaw) {
  yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

} // ns utils
} // ns ikf
