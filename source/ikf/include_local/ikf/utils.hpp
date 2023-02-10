/******************************************************************************
* FILENAME:     utils.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     18.04.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_UTILS_HPP
#define IKF_UTILS_HPP
#include <ikf/types.hpp>
#include <Eigen/Eigen>

namespace ikf
{


  /**
  * @brief conversion and print functions for Eigen, and OpenCV
  */
  namespace utils
  {

    static inline std::string print(Eigen::Quaternion<Precision_t> const& q)
    {
      std::stringstream str;
      str << " (w,x,y,z) " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
      return str.str();
    }

    static inline std::string print(Eigen::Matrix<Precision_t, 3, 1> const& p)
    {
      std::stringstream str;
      str << " (x,y,z) " << p.x() << " " << p.y() << " " << p.z();
      return str.str();
    }
  } // namespace utils

} // namespace ikf

#endif // IKF_UTILS_HPP
