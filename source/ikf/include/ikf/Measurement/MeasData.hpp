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
#include <iomanip>      // std::setprecision
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
    size_t id_sensor = 0;
    std::string meas_type;
    std::string meta_info;
    eObservationType obs_type = eObservationType::UNKNOWN;
    Eigen::VectorXd z; // measurement at t_m
    Eigen::MatrixXd R; // measurement covariance at t_m

    friend std::ostream& operator<< (std::ostream& out, const MeasData& obj)
    {
      out << "MeasData:";
      out << std::left;
      out << " t_m=" << std::setw(16) << obj.t_m.str();
      out << ", t_p=" << std::setw(16) << obj.t_p.str();
      out << ", ID=" << std::setw(3)<< obj.id_sensor;
      out << ", meas_type=" << std::left << std::setw(20) << obj.meas_type;
      out << ", meta info= "<< std::left << std::setw(12) << obj.meta_info;
      out << ", obs. type=" << std::left  << std::setw(2) << (int)obj.obs_type;
      out << ", z=" << std::setprecision(4) <<  obj.z;
      out << ", R=" << std::setprecision(4) << obj.R.diagonal().transpose();
      out << std::internal;
      return out;
    }

  };


}

#endif // MEASDATA_HPP
