/******************************************************************************
* FILENAME:     SimpleLinearIKF.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     06.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef SIMPLELINEARIKF_HPP
#define SIMPLELINEARIKF_HPP
#include <ikf/ikf_api.h>
#include <ikf/IKF/IIsolatedKalmanFilter.hpp>
#include <ikf/IKF/LinearBelief.hpp>


namespace ikf {


class IKF_API SimpleLinearIKF: public IIsolatedKalmanFilter {
public:
  SimpleLinearIKF(bool const handle_delayed_meas=true , double const horizon_sec=1.0);
  SimpleLinearIKF(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, std::string const &name, size_t const ID, bool const handle_delayed_meas=true , double const horizon_sec=1.0 );
  ~SimpleLinearIKF();

  void define_system(Eigen::MatrixXd const& F, Eigen::MatrixXd const& G, Eigen::MatrixXd const& Q, Eigen::MatrixXd const& H, Eigen::MatrixXd const& R);

  // based on system definition:
  bool propagate(double const dt);
  bool propagate(double const dt, Eigen::VectorXd const& u, Eigen::VectorXd const& var_u);
  bool update(Eigen::VectorXd const& z, Timestamp const& t);

  // propagation based on state transtion and process noise:
  bool propagate(Eigen::MatrixXd const& F, Eigen::MatrixXd const& Q, double const dt);
  // propagation based on noisy control input
  bool propagate(Eigen::MatrixXd const& F, Eigen::MatrixXd const& Q, double const dt, Eigen::MatrixXd const& G, Eigen::VectorXd const& u, Eigen::VectorXd const& var_u);

  bool update(Eigen::MatrixXd const& H, Eigen::MatrixXd const& R, Eigen::VectorXd const& z, Timestamp const& t);


  bool update_joint(Eigen::MatrixXd const& H_II, Eigen::MatrixXd const& H_JJ, size_t const ID_J, Eigen::MatrixXd const& R, Eigen::VectorXd const& z, Timestamp const& t);


  bool plot_estimate();
protected:
  // IFilterInstance interface
  ProcessMeasResult_t progapation_measurement(const MeasData &m) override;
  ProcessMeasResult_t local_private_measurement(const MeasData &m) override;

  // IIsolatedKalmanFilter interface
  ProcessMeasResult_t local_joint_measurement(const MeasData &m) override;

  bool m_defined = false;
  Eigen::MatrixXd m_F, m_Q, m_G, m_R; // propagation
  Eigen::MatrixXd m_H_priv; // private observation
  Eigen::MatrixXd m_H_joint_ii, m_H_joint_jj; // local joint observation via IsolatedKalmanFilterHandler
};

} // ns mmsf
#endif // SIMPLELINEARIKF_HPP
