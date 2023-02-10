/******************************************************************************
* FILENAME:     ProcessMeasResult_t.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     30.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef PROCESSMEASRESULT_T_HPP
#define PROCESSMEASRESULT_T_HPP
#include <string>
#include <eigen3/Eigen/Eigen>


namespace ikf {

struct ProcessMeasResult_t {
  bool rejected = true;
  Eigen::VectorXd residual;
  bool skipped = false;
  std::string observation_type;
  std::vector<size_t> ID_participants;
};


} //namespace ikf
#endif // PROCESSMEASRESULT_T_HPP
