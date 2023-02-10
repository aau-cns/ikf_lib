/******************************************************************************
* FILENAME:     LinearIKF.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     03.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef LINEARIKF_HPP
#define LINEARIKF_HPP
#include <ikf/ikf_api.h>
#include <ikf/IKF/IIsolatedKalmanFilter.hpp>
#include <ikf/IKF/LinearBelief.hpp>
namespace ikf {


class IKF_API LinearIKF: public IIsolatedKalmanFilter {
public:
  LinearIKF() = delete;
  LinearIKF(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, std::string const &name, size_t const ID, bool const handle_delayed_meas=true , double const horizon_sec=1.0);
  ~LinearIKF();

  void define_system(Eigen::MatrixXd const& F, Eigen::MatrixXd const& G, Eigen::MatrixXd const& Q, Eigen::MatrixXd const& H);

protected:
  // IFilterInstance interface
  ProcessMeasResult_t progapation_measurement(const MeasData &m) override;
  ProcessMeasResult_t local_private_measurement(const MeasData &m) override;

  // IIsolatedKalmanFilter interface
  ProcessMeasResult_t local_joint_measurement(const MeasData &m) override;

  bool m_defined = false;
  Eigen::MatrixXd m_F, m_Q, m_G; // propagation
  Eigen::MatrixXd m_H_priv; // private observation
  Eigen::MatrixXd m_H_joint_ii, m_H_joint_jj; // local joint observation via IsolatedKalmanFilterHandler
};

} // ns mmsf

#endif // LINEARIKF_HPP
