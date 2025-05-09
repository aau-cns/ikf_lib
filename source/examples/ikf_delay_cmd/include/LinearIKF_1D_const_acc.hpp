/******************************************************************************
* FILENAME:     LinearIKF.hpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
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
#ifndef LINEARIKF_HPP
#define LINEARIKF_HPP
#include "ikf/Estimator/IIsolatedKalmanFilter.hpp"
#include <ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp>
#include <ikf/utils/eigen_utils.hpp>
#include <iostream>

class LinearIKF_1D_const_acc : public ikf::IIsolatedKalmanFilter {
public:
  LinearIKF_1D_const_acc(std::shared_ptr<ikf::IsolatedKalmanFilterHandler> pHandler, size_t const ID) : ikf::IIsolatedKalmanFilter(pHandler, ID) {
    std::cout << "LinearIKF_1D_const_acc: 1D constant acceleration moving body (harmonic motion) ID=" << ID << std::endl;
  }
  virtual ~LinearIKF_1D_const_acc() {}

  // IKalmanFilter interface
public:
  ikf::ProcessMeasResult_t progapation_measurement(const ikf::MeasData &m) override {
    ikf::ProcessMeasResult_t res;
    int dim_x = 2; // Number of states
    int dim_u = 1; // Number of inputs
    Eigen::MatrixXd Phi_ab(dim_x, dim_x); // System dynamics matrix
    Eigen::MatrixXd G_a(dim_x, dim_u); // Control input matrix
    Eigen::MatrixXd Q_II_ab(dim_x, dim_x); // Process noise covariance


    ikf::pBelief_t bel_a;
    ikf::Timestamp t_a;
    ikf::Timestamp t_b = m.t_m;
    if(!get_belief_before_t(t_b, bel_a, t_a)) {
      std::cout << "ERROR: apply_propagation belief before at t=" << t_b << std::endl;
      res.status = ikf::eMeasStatus::DISCARED;
    }
    double const dt = t_b.to_sec() - t_a.to_sec(); // Time step

    // see: https://www.kalmanfilter.net/modeling.html (Example continued: constant acceleration moving body)
    // Discrete LTI projectile motion, measuring position only
    Phi_ab << 1, dt, 0, 1;
    G_a << 0.5*dt*dt, dt;

    // Reasonable covariance matrices
    Q_II_ab = Q_II_ab.Identity(2,2)*0.000001;

    Eigen::VectorXd mean_b = (Phi_ab * bel_a->mean() + G_a * m.z);
    Eigen::MatrixXd Q_ab = G_a * m.R * G_a.transpose() + Q_II_ab;

    if(!apply_propagation(bel_a, mean_b, Phi_ab, Q_ab, t_a, t_b)) {
      std::cout << "ERROR: apply_propagation failed at t=" << t_b << std::endl;
    } else {
      res.status = ikf::eMeasStatus::REJECTED;
    }

    res.obs_type = ikf::eObservationType::PROPAGATION;  // "control_input_acc";
    res.status = ikf::eMeasStatus::PROCESSED;

    return res;
  }
  ikf::ProcessMeasResult_t local_private_measurement(const ikf::MeasData &m) override {
    ikf::ProcessMeasResult_t res;
    int dim_z = 1; // Number of measurements
    int dim_x = 2; // Number of states
    Eigen::MatrixXd H_private(dim_z, dim_x); // Output matrix
    H_private << 1, 0;

    ikf::pBelief_t bel;
    ikf::Timestamp t = m.t_m;
    if(!get_belief_at_t(t, bel)) {
      return res;
    }

    Eigen::VectorXd r = m.z - H_private * bel->mean();
    res.meas_type = "local_position";
    res.residual = r;
    ikf::KalmanFilter::CorrectionCfg_t cfg;

// choose an API for private observations:
#if 0
    bool rejected = !apply_private_observation(bel, H_private, m.R, r, t, cfg);
#else
    std::map<size_t, Eigen::MatrixXd> dict_H = {{m_ID, H_private}};
    bool rejected = !m_pHandler->apply_observation(dict_H, m.R, r, t, cfg);
#endif

    res.status = ikf::eMeasStatus::PROCESSED;
    if (rejected) {
      res.status = ikf::eMeasStatus::REJECTED;
    }
    return res;

  }

  // IIsolatedKalmanFilter interface
public:
  ikf::ProcessMeasResult_t local_joint_measurement(const ikf::MeasData &m) override {
    ikf::ProcessMeasResult_t res;
    int dim_z = 1; // Number of measurements
    int dim_x = 2; // Number of states
    Eigen::MatrixXd H_II(dim_z, dim_x); // Output matrix
    Eigen::MatrixXd H_JJ(dim_z, dim_x); // Output matrix
    H_II << -1, 0;
    H_JJ << 1, 0;


    size_t ID_J = std::stoi(m.meta_info);
    res.ID_participants.push_back(ID_J);
    res.meas_type = m.meas_type + " between [" + std::to_string(m_ID) + "," + m.meta_info + "]";
    ikf::KalmanFilter::CorrectionCfg_t cfg;

// choose an API for private observations
#if 0
    bool rejected = !m_pHandler->apply_joint_observation(m_ID, ID_J, H_II, H_JJ, m.R, m.z, m.t_m, cfg);
#else
    std::map<size_t, Eigen::MatrixXd> dict_H = {{m_ID, H_II}, {ID_J, H_JJ}};
    Eigen::VectorXd z_est = ikf::utils::horcat(H_II, H_JJ)
                            * ikf::utils::vertcat_vec(this->get_belief_at_t(m.t_m)->mean(),
                                                      m_pHandler->get(ID_J)->get_belief_at_t(m.t_m)->mean());
    Eigen::VectorXd r = m.z - z_est;

    bool rejected = !m_pHandler->apply_observation(dict_H, m.R, r, m.t_m, cfg);
#endif

    res.status = ikf::eMeasStatus::PROCESSED;
    if (rejected) {
      res.status = ikf::eMeasStatus::REJECTED;
    }
    return res;
  }

  // IKalmanFilter interface
protected:
  virtual bool predict_to(const ikf::Timestamp &t_b) override { return false; }
};

#endif  // LINEARIKF_HPP
