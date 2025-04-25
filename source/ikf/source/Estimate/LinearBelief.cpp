/******************************************************************************
* FILENAME:     LinearBelief.cpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     06.02.2023
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
#include <ikf/Estimate/LinearBelief.hpp>
#include <ikf/Estimator/KalmanFilter.hpp>

namespace ikf {

LinearBelief::LinearBelief() : IBelief() {}

//LinearBelief::LinearBelief(Eigen::VectorXd mean, Eigen::MatrixXd Sigma, Timestamp t) : IBelief(mean, Sigma, t) {}

LinearBelief::~LinearBelief() {}

std::shared_ptr<IBelief> LinearBelief::clone() {
  auto p_bel = std::make_shared<LinearBelief>(LinearBelief(*this));
  //  p_bel->set(this->mean(), this->Sigma());
  //  p_bel->set_timestamp(this->timestamp());
  return p_bel;
}

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
