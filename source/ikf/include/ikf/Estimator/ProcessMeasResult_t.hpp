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
  bool skipped = false;
  Eigen::VectorXd residual;
  std::string observation_type;
  std::vector<size_t> ID_participants;

  friend std::ostream& operator<< (std::ostream& out, const ProcessMeasResult_t& obj)
  {
    out << "MeasResult: " << "rejected=:" << obj.rejected << ", skipped=" << obj.skipped;
    out << ", residual:" << obj.residual << ", observation_type=" << obj.observation_type;
    out << ", num participants: " << obj.ID_participants.size();
    return out;
  }
};


} //namespace ikf
#endif // PROCESSMEASRESULT_T_HPP
