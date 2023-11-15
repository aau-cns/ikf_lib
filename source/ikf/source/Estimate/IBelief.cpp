/******************************************************************************
* FILENAME:     IBelief.cpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
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
#include <ikf/Estimate/IBelief.hpp>
#include <ikf/Estimator/KalmanFilter.hpp>

namespace ikf {

IBelief::IBelief() {}

/*
IBelief::IBelief(Eigen::VectorXd mean, Eigen::MatrixXd Sigma, Timestamp timestamp) :
    m_mean(mean), m_Sigma(Sigma), m_timestamp(timestamp) {
  m_ns_dim = m_mean.rows();
  m_es_dim = m_Sigma.rows();
}
*/

IBelief::~IBelief() {}

const Eigen::VectorXd& IBelief::mean() const {
  return m_mean;
}

const Eigen::MatrixXd &IBelief::Sigma() const {
  return m_Sigma;
}

void IBelief::mean(const Eigen::VectorXd &vec) {
  m_mean = vec;
}

void IBelief::Sigma(const Eigen::MatrixXd &Cov) {
  m_Sigma = Cov;
}

bool IBelief::set(const Eigen::VectorXd &mean, const Eigen::MatrixXd &Sigma) {
  if (!KalmanFilter::check_dim(mean, Sigma)) { return false; }
  this->mean(mean);
  this->Sigma(Sigma);
  return true;
}

IBelief &IBelief::operator =(const Eigen::VectorXd &param) {
  this->mean(param);
  return *this;
}

IBelief &IBelief::operator =(const Eigen::MatrixXd &param) {
  this->Sigma(param);
  return *this;
}

size_t IBelief::es_dim() const { return m_es_dim; }

size_t IBelief::ns_dim() const { return m_ns_dim; }

const Timestamp &IBelief::timestamp() const { return m_timestamp; }

void IBelief::set_timestamp(const Timestamp &t) { m_timestamp = t; }

void IBelief::set_options(const BeliefOptions &o) {
  m_options = o;
}

Eigen::VectorXd IBelief::boxminus(std::shared_ptr<IBelief> right) { return m_mean - right->mean(); }

void IBelief::boxplus(const Eigen::VectorXd &dx) { correct(dx); }

void IBelief::correct(const Eigen::VectorXd &dx)  // inplace and accoring to the error definiton!
{
  m_mean += dx;
}

void IBelief::correct(const Eigen::VectorXd &dx,
                      const Eigen::MatrixXd &Sigma_apos)  // inplace and accoring to the error definiton!
{
  correct(dx);
  Sigma(Sigma_apos);
}

Eigen::MatrixXd IBelief::plus_jacobian(const Eigen::VectorXd &dx) {
  return Eigen::MatrixXd::Identity(m_es_dim, m_es_dim);
}

const BeliefOptions &IBelief::options() const { return m_options; }

void IBelief::apply_init_strategy(std::shared_ptr<IBelief> &bel_0, const eInitStrategies type, const int seed) {
  if (seed != 0) {
    // set the seed of the random variable generator to something...
    // TODO: we need a multivariate normal random generator!
  }
}


}
