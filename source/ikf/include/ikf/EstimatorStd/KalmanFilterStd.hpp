/******************************************************************************
* FILENAME:     KalmanFilterStd.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
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
  KalmanFilterStd(ptr_belief const& belief);

  KalmanFilterStd(ptr_belief const& belief, Timestamp const& t);

  void set_belief(ptr_belief p_bel);
  ptr_belief get_belief();
  virtual ~KalmanFilterStd();

  virtual bool propagate(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_b);

  virtual bool propagate(Eigen::MatrixXd const& Phi_II_ab, Eigen::MatrixXd const& Q_II_ab,  const Timestamp &t_b, Eigen::MatrixXd const& G_a, Eigen::VectorXd const& u_a, Eigen::VectorXd const& var_u);

  virtual bool private_update(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                 const Eigen::VectorXd &z);

protected:
  ptr_belief m_belief;
};

} // ns ifk
#endif // KALMANFILTERSTD_HPP
