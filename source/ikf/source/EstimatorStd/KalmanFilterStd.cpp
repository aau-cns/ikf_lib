/******************************************************************************
* FILENAME:     KalmanFilterStd.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
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
#include "ikf/utils/RTVerification.hpp"
#include "ikf/utils/eigen_utils.hpp"
#include <ikf/EstimatorStd/KalmanFilterStd.hpp>
#include <ikf/Estimator/KalmanFilter.hpp>
namespace ikf {

KalmanFilterStd::KalmanFilterStd() {}

KalmanFilterStd::KalmanFilterStd(const ptr_belief &belief) {
  m_belief = belief->clone();
}

KalmanFilterStd::KalmanFilterStd(const ptr_belief &belief, const Timestamp &t) {
  m_belief = belief->clone();
  m_belief->set_timestamp(t);
}

void KalmanFilterStd::set_belief(ptr_belief p_bel) { m_belief = p_bel; }

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

bool KalmanFilterStd::propagate(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_b, const Eigen::MatrixXd &G_a, const Eigen::VectorXd &u_a, const Eigen::VectorXd &var_u) {
  if (KalmanFilter::check_dim(m_belief->Sigma(), Phi_II_ab, Q_II_ab) &&
      m_belief->timestamp() < t_b)
  {

    Eigen::VectorXd mean_b = (Phi_II_ab * m_belief->mean() + G_a * u_a);
    Eigen::MatrixXd Q_d = G_a * var_u * G_a.transpose() + Q_II_ab;

    m_belief->Sigma(KalmanFilter::covariance_propagation(m_belief->Sigma(), Phi_II_ab, Q_d));
    m_belief->mean(mean_b);
    m_belief->set_timestamp(t_b);
    return true;
  }
  return false;
}

bool KalmanFilterStd::private_update(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &z) {
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(m_belief->Sigma()), "Apri covariance is not PSD!");

  Eigen::VectorXd r = z - H_II * m_belief->mean();
  KalmanFilter::CorrectionCfg_t cfg;
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_II, R, r, m_belief->Sigma(), cfg);
  if (!res.rejected) {
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(res.Sigma_apos), "Apos covariance is not PSD!");

    // inplace correction, no need to write belief in HistBelief
    m_belief->correct(res.delta_mean, res.Sigma_apos);
  }
  return !res.rejected;
}


} // ns ikf
