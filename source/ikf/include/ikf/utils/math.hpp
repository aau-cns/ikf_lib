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
#include <math.h>
#include <ikf/ikf_api.h>
#include <Eigen/Dense>

namespace ikf{
namespace utils {

  //template<typename Derived=double>
  double IKF_API rad2deg(double const rad);
  //template<typename Derived=double>
  double IKF_API deg2rad(double const deg);

  // https://github.com/aau-cns/mars_lib/blob/main/source/mars/source/utils.cpp
  Eigen::Matrix3d IKF_API skew(const Eigen::Vector3d& v);
  Eigen::Vector3d IKF_API skew_inv(const Eigen::Matrix3d& M);
  Eigen::Matrix4d IKF_API OmegaMat(const Eigen::Vector3d& v);
  Eigen::Matrix4d IKF_API MatExp(const Eigen::Matrix4d& A, const int order=4);

} // ns utils
} // ns ikf
#endif // IKF_UTILS_MATH_HPP
