/******************************************************************************
* FILENAME:     LinearIKF.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     06.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/IKF/LinearIKF.hpp>
namespace ikf {


LinearIKF::LinearIKF(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, const std::string &name, const size_t ID, const bool handle_delayed_meas, const double horizon_sec): IIsolatedKalmanFilter(ptr_Handler, name, ID, handle_delayed_meas, horizon_sec) {}

LinearIKF::~LinearIKF() {}

void LinearIKF::define_system(const Eigen::MatrixXd &F, const Eigen::MatrixXd &G, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &H) {
  m_F = F; m_G = G; m_Q = Q; m_H_priv = H;
  m_defined = true;
}

ProcessMeasResult_t LinearIKF::progapation_measurement(const MeasData &m) {
  Timestamp t_a = current_t();
  ptr_belief p_bel_a = get_belief_at_t(t_a);

  Timestamp t_b = m.t_m;

  double dt = t_b.to_sec() - t_a.to_sec();
  RTV_EXPECT_TRUE_MSG(dt > 0.0, "Measurement must be ahead of the current state at t_a" + t_a.str());


  Eigen::VectorXd mean_b = (m_F * p_bel_a->mean() + m_G * m.z);

  Eigen::MatrixXd Q = m_G * m.R * m_G.transpose() + m_Q;

  ProcessMeasResult_t res;
  res.rejected = apply_propagation(p_bel_a, mean_b, m_F, Q, t_a, t_b);

  return res;
}

ProcessMeasResult_t LinearIKF::local_private_measurement(const MeasData &m) {
  Timestamp t_a = m.t_m;
  ptr_belief p_bel_a = get_belief_at_t(m.t_m);
  if(p_bel_a){
    Eigen::VectorXd r;
    r.setZero(1,1);
    r << m.z - m_H_priv*p_bel_a->mean();


    ProcessMeasResult_t res;
    res.rejected = apply_private_observation(p_bel_a, m_H_priv, m.R, r, m.t_m);
    return res;
  }
  return ProcessMeasResult_t();
}

ProcessMeasResult_t LinearIKF::local_joint_measurement(const MeasData &m) {
  return ProcessMeasResult_t();
}



} // ns mmsf
