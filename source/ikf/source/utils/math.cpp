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


} // ns utils
} // ns ikf
