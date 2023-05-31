/******************************************************************************
* FILENAME:     MeasData.hpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         <roland.jung@ieee.org>
* VERSION:      v0.0.0
* CREATION:     25.01.2023
*
* Copyright (C) 2023 Roland Jung, Control of Networked Systems, University of Klagenfurt, Austria.
*
* All rights reserved.
*
* This software is licensed under the terms of the BSD-2-Clause-License with
* no commercial use allowed, the full terms of which are made available
* in the LICENSE file. No license in patents is granted.
*
* You can contact the author at <roland.jung@aau.at>
******************************************************************************/
#ifndef MEASDATA_HPP
#define MEASDATA_HPP
#include <unordered_map>
#include <memory>
#include <iomanip>      // std::setprecision
#include <ikf/Container/TTimeHorizonBuffer.hpp>
#include <ikf/utils/RTVerification.hpp>
#include <Eigen/Dense>

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

    bool has_meas_noise() {
      return R.size();
    }
    friend std::ostream& operator<< (std::ostream& out, const MeasData& obj)
    {
      out << "MeasData:";
      out << std::left;
      out << " t_m=" << std::setw(16) << obj.t_m.str();
      out << ", t_p=" << std::setw(16) << obj.t_p.str();
      out << ", ID=" << std::setw(3)<< obj.id_sensor;
      out << ", meas_type=" << std::left << std::setw(20) << obj.meas_type;
      out << ", meta info="<< std::left << std::setw(12) << obj.meta_info;
      out << ", obs. type=" << std::left  << std::setw(2) << (int)obj.obs_type;
      out << ", z=" << std::setprecision(4) <<  obj.z.transpose();
      out << ", R=" << std::setprecision(4) << obj.R.diagonal().transpose();
      out << std::internal;
      return out;
    }

  };


}

#endif // MEASDATA_HPP
