/******************************************************************************
* FILENAME:     KalmanFilterStd.hpp
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
#ifndef KALMANFILTERSTD_HPP
#define KALMANFILTERSTD_HPP
#include <ikf/ikf_api.h>
#include <ikf/Estimate/IBelief.hpp>
namespace ikf {

///
/// \brief The KalmanFilterStd class
/// Naive implementation of the Kalman Filter without buffering, thus no Out-Of-Sequence support.
///
class IKF_API KalmanFilterStd {
public:
  KalmanFilterStd();
  KalmanFilterStd(pBelief_t const& belief);

  KalmanFilterStd(pBelief_t const& belief, Timestamp const& t);

  void set_belief(pBelief_t p_bel);
  pBelief_t get_belief();
  virtual ~KalmanFilterStd();

  virtual bool propagate(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_b);

  virtual bool propagate(Eigen::MatrixXd const& Phi_II_ab, Eigen::MatrixXd const& Q_II_ab,  const Timestamp &t_b, Eigen::MatrixXd const& G_a, Eigen::VectorXd const& u_a, Eigen::VectorXd const& var_u);

  virtual bool private_update(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                 const Eigen::VectorXd &z);

protected:
  pBelief_t m_belief;
};

} // ns ifk
#endif // KALMANFILTERSTD_HPP
