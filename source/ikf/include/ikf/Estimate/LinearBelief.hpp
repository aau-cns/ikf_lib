/******************************************************************************
* FILENAME:     LinearBelief.hpp
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
#ifndef LINEARBELIEF_HPP
#define LINEARBELIEF_HPP
#include <ikf/ikf_api.h>
#include <ikf/Estimate/IBelief.hpp>

namespace ikf {

class IKF_API LinearBelief: public IBelief {
  // IBelief interface
public:
  LinearBelief();
  LinearBelief(Eigen::VectorXd mean, Eigen::MatrixXd Sigma, Timestamp t);
  ~LinearBelief();

  ////////////////////////////////////////////////////////////
  //// PURE VIRTUAL:
  std::shared_ptr<IBelief> clone() override;
  std::shared_ptr<IBelief> interpolate(std::shared_ptr<IBelief> obj_a, std::shared_ptr<IBelief> obj_b, const double i) override;
  void correct(const Eigen::VectorXd &dx) override;
  void correct(const Eigen::VectorXd &dx, const Eigen::MatrixXd &Sigma_apos) override;
  //// PURE VIRTUAL:
  ////////////////////////////////////////////////////////
  size_t es_dim() const override;
  size_t ns_dim() const override;
};

} // ns mmsf

#endif // LINEARBELIEF_HPP
