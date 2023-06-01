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

namespace ikf{
namespace utils {

  //template<typename Derived=double>
  static inline double rad2deg(double const rad) {
    return rad * (180/M_PI);
  }
  //template<typename Derived=double>
  static inline double deg2rad(double const deg) {
    return deg * (M_PI/180);
  }

} // ns utils
} // ns ikf
#endif // MATH_HPP
