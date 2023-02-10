/******************************************************************************
* FILENAME:     types.hpp
* PURPOSE:
* AUTHOR:       %USER%
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     %DATE%
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_TYPES_HPP
#define IKF_TYPES_HPP
#include <ikf/ikf_api.h>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <stdint.h>

namespace ikf
{
  // pollute the mmsf namespace only with selected types!
  // TODO: make decision if it is acceptable at all!

  typedef double Precision_t;
  typedef Eigen::Quaternion<Precision_t> Quaternion_t;
  typedef Eigen::Matrix<Precision_t,3,1> Vector3_t;
  typedef Eigen::Matrix<Precision_t,3,3> Mat3_t;
  typedef Eigen::Matrix<Precision_t,4,4> Mat4_t;


  class IKF_API IFormatable
  {
      virtual void format(std::ostream& os) = 0;
  }; // class IFormatable

} // namespace ikf

#endif // IKF_HELPER_HPP
