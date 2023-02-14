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
#include "ikf/EstimatorStd/IsolatedKalmanFilterStd.hpp"
#include <iostream>


class LinearIKF: public ikf::IsolatedKalmanFilterStd {
public:
  LinearIKF(std::shared_ptr<ikf::IKFHandlerStd> pHandler, size_t const ID) : ikf::IsolatedKalmanFilterStd(pHandler, ID) {}
  virtual ~LinearIKF() {}

  void define_system(Eigen::MatrixXd const& F, Eigen::MatrixXd const& G, Eigen::MatrixXd const& Q, Eigen::MatrixXd const& H, Eigen::MatrixXd const& R, double const dt) {
    m_defined = true;
    m_F = F; m_G = G; m_Q = Q; m_H_priv = H; m_R = R;
    m_dt = dt;
  }

  // based on system definition:
  bool predict() {
    if (m_defined) {
      ikf::Timestamp t_b(m_belief->timestamp().to_sec() + m_dt);
      return ikf::IsolatedKalmanFilterStd::propagate(m_F, m_Q, t_b);
    }
    std::cout << "System not defined!" << std::endl;
    return false;
  }

  bool predict(Eigen::VectorXd const& u, Eigen::VectorXd const& var_u) {
    if(m_defined) {
      ikf::Timestamp t_b(m_belief->timestamp().to_sec() + m_dt);
      return ikf::IsolatedKalmanFilterStd::propagate(m_F, m_Q, t_b, m_G, u, var_u);
    }
    return false;
  }


  // propagation based on state transtion and process noise:
  // propagation based on noisy control input
  bool predict(Eigen::MatrixXd const& F, Eigen::MatrixXd const& Q, Eigen::MatrixXd const& G, Eigen::VectorXd const& u, Eigen::VectorXd const& var_u) {
    ikf::Timestamp t_b(m_belief->timestamp().to_sec() + m_dt);
    return ikf::IsolatedKalmanFilterStd::propagate(F, Q, t_b, G, u, var_u);
  }

  bool update(Eigen::VectorXd const& z) {
    if (m_defined) {
      return ikf::IsolatedKalmanFilterStd::private_update(m_H_priv, m_R, z);
    }
    return false;
  }


  bool joint_update(Eigen::MatrixXd const& H_II, Eigen::MatrixXd const& H_JJ, size_t const ID_J, Eigen::MatrixXd const& R, Eigen::VectorXd const& z) {
    return ikf::IsolatedKalmanFilterStd::joint_update(m_ID, ID_J, H_II, H_JJ, R, z);
  }

  bool m_defined = false;
  double m_dt = 0.0;
  Eigen::MatrixXd m_F, m_Q, m_G, m_R; // propagation
  Eigen::MatrixXd m_H_priv; // private observation
  Eigen::MatrixXd m_H_joint_ii, m_H_joint_jj; // local joint observation via IsolatedKalmanFilterHandler
};


#endif // LINEARIKF_HPP
