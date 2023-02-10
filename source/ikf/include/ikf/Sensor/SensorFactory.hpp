/******************************************************************************
* FILENAME:     SensorFactory.hpp
* PURPOSE:      SensorFactory
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     21.06.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_SENSORFACTORY_HPP
#define IKF_SENSORFACTORY_HPP

#include <ikf/ikf_api.h>

namespace ikf
{

// https://gist.github.com/mbains/3406184
// https://gist.github.com/sacko87/3359911
  class IKF_API SensorFactory
  {
    public:

      static SensorFactory& get_instance();
      // delete copy and move constructors and assign operators
      SensorFactory(SensorFactory const&) = delete;             // Copy construct
      SensorFactory(SensorFactory&&)      = delete;             // Move construct
      SensorFactory& operator=(SensorFactory const&) = delete;  // Copy assign
      SensorFactory& operator=(SensorFactory&&) = delete;       // Move assign


      enum class eSensors:char
      {
        Pose=0,
        Position,
        BodyVelocity
      };



    protected:
      SensorFactory();
      ~SensorFactory();
  };

} // namespace ikf
#endif // IKF_SENSORFACTORY_HPP
