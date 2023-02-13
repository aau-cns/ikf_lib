/******************************************************************************
* FILENAME:     KalmanFilterStd.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/EstimatorStd/KalmanFilterStd.hpp>
#include <ikf/Estimator/KalmanFilter.hpp>
namespace ikf {

KalmanFilterStd::KalmanFilterStd(const ptr_belief &belief) {
  m_belief = belief->clone();
}

KalmanFilterStd::KalmanFilterStd(const ptr_belief &belief, const Timestamp &t) {
  m_belief = belief->clone();
  m_belief->set_timestamp(t);
}

ptr_belief KalmanFilterStd::get_belief() { return m_belief; }

KalmanFilterStd::~KalmanFilterStd() {}

bool KalmanFilterStd::propagate(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_b) {
  if (KalmanFilter::check_dim(m_belief->Sigma(), Phi_II_ab, Q_II_ab) &&
      m_belief->timestamp() < t_b)
  {
    m_belief->Sigma(KalmanFilter::covariance_propagation(m_belief->Sigma(), Phi_II_ab, Q_II_ab));
    m_belief->mean(Phi_II_ab * m_belief->mean());
    m_belief->set_timestamp(t_b);
    return true;
  }
  return false;
}

bool KalmanFilterStd::private_update(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &z) {
  Eigen::VectorXd r = z - H_II * m_belief->mean();
  KalmanFilter::CorrectionCfg_t cfg;
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_II, R, r, m_belief->Sigma(), cfg);
  if (!res.rejected) {
    // inplace correction, no need to write belief in HistBelief
    m_belief->correct(res.delta_mean, res.Sigma_apos);
  }
  return !res.rejected;
}


} // ns ikf
