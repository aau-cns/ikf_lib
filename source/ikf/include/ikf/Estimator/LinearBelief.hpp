/******************************************************************************
* FILENAME:     LinearBelief.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     06.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef LINEARBELIEF_HPP
#define LINEARBELIEF_HPP
#include <ikf/ikf_api.h>
#include <ikf/Estimator/IIsolatedKalmanFilter.hpp>

namespace ikf {

class IKF_API LinearBelief: public IBelief {
  // IBelief interface
public:
  LinearBelief();
  ~LinearBelief();
  Eigen::VectorXd mean() override;
  Eigen::MatrixXd Sigma() override;
  void mean(const Eigen::VectorXd &vec) override;
  void Sigma(const Eigen::MatrixXd &Cov) override;
  bool set(const Eigen::VectorXd &mean, const Eigen::MatrixXd &Sigma) override;
  IBelief &operator =(const Eigen::VectorXd &param) override;
  IBelief &operator =(const Eigen::MatrixXd &param) override;
  std::shared_ptr<IBelief> clone() override;

  std::shared_ptr<IBelief> interpolate(std::shared_ptr<IBelief> obj_a, std::shared_ptr<IBelief> obj_b, const double i) override;
  void correct(const Eigen::VectorXd &dx) override;
  void correct(const Eigen::VectorXd &dx, const Eigen::MatrixXd &Sigma_apos) override;

  size_t es_dim() const override;
  size_t ns_dim() const override;
protected:
  Eigen::VectorXd m_mean;
  Eigen::MatrixXd m_Sigma;
};

} // ns mmsf

#endif // LINEARBELIEF_HPP
