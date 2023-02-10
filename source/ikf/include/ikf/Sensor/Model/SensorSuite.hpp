/******************************************************************************
* FILENAME:     SensorSuite.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef SENSORSUITE_HPP
#define SENSORSUITE_HPP
#include <memory>
#include <unordered_map>
namespace ikf {
class ISensorModel;

class SensorSuite {


protected:
  std::unordered_map<size_t, std::shared_ptr<ISensorModel>> DictSensorsModels;
};

} // mmsf
#endif // SENSORSUITE_HPP
