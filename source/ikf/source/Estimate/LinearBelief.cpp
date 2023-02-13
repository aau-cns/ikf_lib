/******************************************************************************
* FILENAME:     LinearBelief.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     06.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Estimate/LinearBelief.hpp>
#include <ikf/Estimator/KalmanFilter.hpp>

namespace ikf {

LinearBelief::LinearBelief() : IBelief() {}

LinearBelief::LinearBelief(Eigen::VectorXd mean, Eigen::MatrixXd Sigma, Timestamp t) : IBelief(mean, Sigma, t) {}

LinearBelief::~LinearBelief() {}

std::shared_ptr<IBelief> LinearBelief::clone() {
  auto p_bel = std::make_shared<LinearBelief>(LinearBelief());
  p_bel->set(this->mean(), this->Sigma());
  return p_bel;
}

std::shared_ptr<IBelief> LinearBelief::interpolate(std::shared_ptr<IBelief> obj_a, std::shared_ptr<IBelief> obj_b, const double i) { return std::shared_ptr<IBelief>(nullptr); }

void LinearBelief::correct(const Eigen::VectorXd &dx) {
  m_mean += dx;
}

void LinearBelief::correct(const Eigen::VectorXd &dx, const Eigen::MatrixXd &Sigma_apos) {
  correct(dx);
  m_Sigma = Sigma_apos;
}

size_t LinearBelief::es_dim() const { return m_Sigma.rows(); }

size_t LinearBelief::ns_dim() const { return m_Sigma.rows(); }


} // ns mmsf
