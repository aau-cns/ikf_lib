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
#ifndef MATH_HPP
#define MATH_HPP
#include <math.h>
#include <ikf/ikf_api.h>
#include <Eigen/Dense>

namespace ikf{
namespace utils {

  //template<typename Derived=double>
  double IKF_API rad2deg(double const rad);
  //template<typename Derived=double>
  double IKF_API deg2rad(double const deg);

  Eigen::Matrix3d IKF_API skew(const Eigen::Vector3d& v);

} // ns utils
} // ns ikf
#endif // MATH_HPP
