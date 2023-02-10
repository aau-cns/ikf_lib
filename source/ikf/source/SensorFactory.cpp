/******************************************************************************
* FILENAME:     SensorFactory.cpp
* PURPOSE:      SensorFactory
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     21.06.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Sensor/SensorFactory.hpp>


namespace ikf
{

  SensorFactory &SensorFactory::get_instance()
  {
    static SensorFactory my_instance;
    return my_instance;
  }

  SensorFactory::SensorFactory()
  {

  }

  SensorFactory::~SensorFactory()
  {

  }

} // namespace ikf
