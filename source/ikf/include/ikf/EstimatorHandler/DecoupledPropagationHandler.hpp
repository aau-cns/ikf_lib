/******************************************************************************
 * FILENAME:     DecoupledPropagationHandler.hpp
 * PURPOSE:      Part of the ikf_lib
 * AUTHOR:       Roland Jung
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     03.02.2023
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
#ifndef DECOUPLEDPROPAGATIONHANDLER_HPP
#define DECOUPLEDPROPAGATIONHANDLER_HPP
#include <ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp>
#include <ikf/ikf_api.h>
#include <memory>
namespace ikf {

class IKF_API DecoupledPropagationHandler : public IsolatedKalmanFilterHandler {
public:
  DecoupledPropagationHandler(double const horizon_sec = 1.0);
  ~DecoupledPropagationHandler() = default;

  virtual bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::MatrixXd& R,
                                 const Eigen::VectorXd& r, const Timestamp& t,
                                 const KalmanFilter::CorrectionCfg_t& cfg) override;
  virtual bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::VectorXd& z,
                                 const Eigen::MatrixXd& R, const Timestamp& t,
                                 const KalmanFilter::CorrectionCfg_t& cfg) override;

  virtual ApplyObsResult_t apply_observation(const Eigen::MatrixXd& R, const Eigen::VectorXd& z, const Timestamp& t,
                                             IIsolatedKalmanFilter::h_joint const& h, std::vector<size_t> const& IDs,
                                             const KalmanFilter::CorrectionCfg_t& cfg) override;

protected:
  virtual Eigen::MatrixXd stack_H(const std::map<size_t, Eigen::MatrixXd>& dict_H) override;

  virtual std::map<size_t, pBelief_t> get_dict_bel(const std::map<size_t, Eigen::MatrixXd>& dict_H,
                                                   Timestamp const& t) override;

};  // class DecoupledPropagationHandler

}  // namespace ikf

#endif  // DECOUPLEDPROPAGATIONHANDLER_HPP
