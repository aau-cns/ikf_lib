#ifndef QUATERNIONUTILS_HPP
#define QUATERNIONUTILS_HPP

#include <ikf/ikf_api.h>
#include <ikf/types.hpp>

namespace ikf
{

  class IKF_API QuaternionUtils
  {
    public:

      /// computes a quaternion from the 3-element small angle approximation theta
      static ikf::Quaternion_t small_angle_to_quaternion(const ikf::Vector3_t & theta)
      {
        const ikf::Precision_t q_squared = theta.squaredNorm() / 4.0;

        if ( q_squared < 1)
        {
          return ikf::Quaternion_t(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
        }
        else
        {
          const ikf::Precision_t w = 1.0 / sqrt(1 + q_squared);
          const ikf::Precision_t f = w*0.5;
          return ikf::Quaternion_t(w, theta[0] * f, theta[1] * f, theta[2] * f);
        }
      }

  };

} //namespace ikf

#endif // QUATERNIONUTILS_HPP
