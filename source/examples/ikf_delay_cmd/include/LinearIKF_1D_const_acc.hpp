/******************************************************************************
* FILENAME:     LinearIKF.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef LINEARIKF_HPP
#define LINEARIKF_HPP
#include "ikf/Estimator/IIsolatedKalmanFilter.hpp"
#include <iostream>


class LinearIKF_1D_const_acc: public ikf::IIsolatedKalmanFilter {
public:
  LinearIKF_1D_const_acc(std::shared_ptr<ikf::IsolatedKalmanFilterHandler> pHandler, size_t const ID) : ikf::IIsolatedKalmanFilter(pHandler, ID) {
      std::cout << "LinearIKF_1D_const_acc: 1D constant acceleration moving body (harmonic motion)" << std::endl;
      std::cout << "see: https://www.kalmanfilter.net/modeling.html (Example continued: constant acceleration moving body)" << std::endl;
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


    ikf::ptr_belief bel_a;
    ikf::Timestamp t_a;
    ikf::Timestamp t_b = m.t_m;
    if(!get_belief_before_t(t_b, bel_a, t_a)) {
      return res;
    }
    double const dt = t_b.to_sec() - t_a.to_sec(); // Time step

    // Discrete LTI projectile motion, measuring position only
    Phi_ab << 1, dt, 0, 1;
    G_a << 0.5*dt*dt, dt;

    // Reasonable covariance matrices
    Q_II_ab = Q_II_ab.Identity(2,2)*0.01;

    Eigen::VectorXd mean_b = (Phi_ab * bel_a->mean() + G_a * m.z);
    Eigen::MatrixXd Q_ab = G_a * m.R * G_a.transpose() + Q_II_ab;

    if(!apply_propagation(bel_a, mean_b, Phi_ab, Q_ab, t_a, t_b)) {

    }

   res.observation_type = "control_input_acc";
   return res;
  }
  ikf::ProcessMeasResult_t local_private_measurement(const ikf::MeasData &m) override {
    ikf::ProcessMeasResult_t res;
    int dim_z = 1; // Number of measurements
    int dim_x = 2; // Number of states
    Eigen::MatrixXd R_private (dim_z,dim_z);  // measurement covariance
    Eigen::MatrixXd H_private(dim_z, dim_x); // Output matrix
    H_private << 1, 0;


    ikf::ptr_belief bel;
    ikf::Timestamp t = m.t_m;
    if(!get_belief_at_t(t, bel)) {
      return res;
    }

    Eigen::VectorXd r = m.z - H_private * bel->mean();
    res.observation_type = "local_position";
    res.residual = r;
    res.rejected = !apply_private_observation(bel, H_private, R_private, r, t);

    return res;

  }

  // IIsolatedKalmanFilter interface
public:
  ikf::ProcessMeasResult_t local_joint_measurement(const ikf::MeasData &m) override {
    ikf::ProcessMeasResult_t res;
    int dim_z = 1; // Number of measurements
    int dim_x = 2; // Number of states
    Eigen::MatrixXd R_private (dim_z,dim_z);  // measurement covariance
    Eigen::MatrixXd H_II(dim_z, dim_x); // Output matrix
    Eigen::MatrixXd H_JJ(dim_z, dim_x); // Output matrix
    H_II << -1, 0;
    H_JJ << 1, 0;


    size_t ID_J = std::stoi(m.meta_info);
    res.ID_participants.push_back(ID_J);
    res.observation_type = m.meas_type + " between [" + std::to_string(m_ID) + "," + m.meta_info + "]";
    res.rejected = !apply_joint_observation(m_ID, ID_J, H_II, H_JJ, m.R, m.z, m.t_m);
    return res;
  }
};


#endif // LINEARIKF_HPP
