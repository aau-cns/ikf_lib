/******************************************************************************
* FILENAME:     MeasData.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     25.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef MEASDATA_HPP
#define MEASDATA_HPP
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <ikf/Container/TTimeHorizonBuffer.hpp>
#include <ikf/utils/RTVerification.hpp>
#include <unordered_map>

namespace ikf
{
  enum class eObservationType {
    UNKNOWN = 0,
    PROPAGATION = 1,
    PRIVATE_OBSERVATION = 2,
    JOINT_OBSERVATION = 3,
  };

  struct MeasData {
    Timestamp t_m; // true measurement timestamp
    Timestamp t_p; // actual processing timestamp
    size_t id_sensor;
    std::string meas_type;
    std::string meta_info;
    eObservationType obs_type = eObservationType::UNKNOWN;
    Eigen::VectorXd z; // measurement at t_m
    Eigen::MatrixXd R; // measurement covariance at t_m

    friend std::ostream& operator<< (std::ostream& out, const MeasData& obj)
    {
      out << "MeasData: " << "t_m=:" << obj.t_m << ", t_p=" << obj.t_p;
      out << ", ID:" << obj.id_sensor << ", meas_type=" << obj.meas_type;
      out << ", meta info: " << obj.meta_info  << ", observation type=" << (int)obj.obs_type;
      out << ", z=" << obj.z << " R=" << obj.R;
      return out;
    }

  };


}

#endif // MEASDATA_HPP
