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
#include <ikf/ikf_api.h>
#include <unordered_map>
#include <memory>
#include <iomanip>      // std::setprecision
#include <ikf/Container/TTimeHorizonBuffer.hpp>
#include <ikf/utils/RTVerification.hpp>
#include <Eigen/Dense>

namespace ikf
{
  enum class IKF_API eObservationType {
    UNKNOWN = 0,
    PROPAGATION = 1,
    PRIVATE_OBSERVATION = 2,
    JOINT_OBSERVATION = 3,
  };

  std::string IKF_API to_string(eObservationType const ot);

  struct IKF_API MeasData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Timestamp t_m; // true measurement timestamp from moment of perception
    Timestamp t_p; // actual processing timestamp for simulation
    size_t id_sensor = 0;
    std::string meas_type;
    std::string meta_info;
    eObservationType obs_type = eObservationType::UNKNOWN;
    Eigen::VectorXd z; // measurement at t_m
    Eigen::MatrixXd R; // measurement covariance at t_m (as vector or matrix -> call get_R()

    bool has_meas_noise() const;

    Eigen::MatrixXd get_R() const;

    friend std::ostream& operator<< (std::ostream& out, const MeasData& obj);

    static MeasData lin_interpolate(MeasData const& m_a, MeasData const& m_c, Timestamp const& t_b);

  };

  // INFO: needs be declared outside again to apply the IKF_API visibility attribute!
  IKF_API std::ostream&  operator<<(std::ostream& os, const MeasData&);


}

#endif // MEASDATA_HPP
