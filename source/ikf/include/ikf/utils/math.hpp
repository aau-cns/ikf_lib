/******************************************************************************
* FILENAME:     math.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     31.05.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_UTILS_MATH_HPP
#define IKF_UTILS_MATH_HPP
#include <Eigen/Dense>
#include <algorithm>
#include <ikf/ikf_api.h>
#include <iterator>
#include <math.h>
#include <numeric>

namespace ikf{
namespace utils {

template <typename T>
T roundn(T const val, size_t const precision) {
  size_t const scale = std::pow(10, precision);
  return std::round(val * scale) / scale;
}

// template<typename Derived=double>
double IKF_API rad2deg(double const rad);
// template<typename Derived=double>
double IKF_API deg2rad(double const deg);

// https://github.com/aau-cns/mars_lib/blob/main/source/mars/source/utils.cpp
Eigen::Matrix3d IKF_API skew(const Eigen::Vector3d& v);
Eigen::Vector3d IKF_API skew_inv(const Eigen::Matrix3d& M);
Eigen::Matrix4d IKF_API OmegaMat(const Eigen::Vector3d& v);
Eigen::Matrix4d IKF_API MatExp(const Eigen::Matrix4d& A, const int order = 4);

// https://stackoverflow.com/a/29871193
/* wrap x -> [0,max) */
double IKF_API wrapMax(double const x, double const max);
/* wrap x -> [min,max) */
double IKF_API wrapMinMax(double const x, double const min, double const max);

double IKF_API wrapToPi(double const x_rad);
double IKF_API wrapTo2Pi(double const x_rad);
double IKF_API wrapTo180deg(double const x_deg);
double IKF_API wrapTo360deg(double const x_deg);

//  R = axang2rot([0,0,1], yaw) * axang2rot([0,1,0], pitch) * axang2rot([1,0,0], roll)
void IKF_API quat2rpy(Eigen::Quaterniond const& q, double& roll, double& pitch, double& yaw);
void IKF_API quat2roll(Eigen::Quaterniond const& q, double& roll);
void IKF_API quat2pitch(Eigen::Quaterniond const& q, double& pitch);
void IKF_API quat2yaw(Eigen::Quaterniond const& q, double& yaw);

// https://stackoverflow.com/questions/7616511/calculate-mean-and-standard-deviation-from-a-vector-of-samples-in-c-using-boos
template <typename Container, typename T = typename std::decay<decltype(*std::begin(std::declval<Container>()))>::type>
T mean(Container&& c) {
  auto b = std::begin(c), e = std::end(c);
  auto size = std::distance(b, e);
  auto sum = std::accumulate(b, e, T());
  return sum / size;
}
template <typename Container, typename T = typename std::decay<decltype(*std::begin(std::declval<Container>()))>::type>
T stddev(Container&& c) {
  auto b = std::begin(c), e = std::end(c);
  auto size = std::distance(b, e);
  auto sum = std::accumulate(b, e, T());
  auto mean = sum / size;
  T accum = T();
  for (const auto d : c) {
    accum += pow(d - mean, 2);
  }
  return std::sqrt(accum / (size - 1));
}

template <typename Container, typename T = typename std::decay<decltype(*std::begin(std::declval<Container>()))>::type>
T max(Container&& c) {
  if (c.empty()) {
    return std::numeric_limits<T>::max();
  } else {
    return *std::max_element(std::begin(c), std::end(c));
  }
}

template <typename Container, typename T = typename std::decay<decltype(*std::begin(std::declval<Container>()))>::type>
T min(Container&& c) {
  if (c.empty()) {
    return std::numeric_limits<T>::max();
  } else {
    return *std::min_element(std::begin(c), std::end(c));
  }
}

}  // namespace utils
} // ns ikf
#endif // IKF_UTILS_MATH_HPP
