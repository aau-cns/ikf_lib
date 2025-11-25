/******************************************************************************
* FILENAME:     SchmidtKalmanFilterHandler.hpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     25.11.2025
*
* Copyright (C) 2025 Roland Jung, Control of Networked Systems, University of Klagenfurt, Austria.
*
* All rights reserved.
*
* This software is licensed under the terms of the BSD-2-Clause-License with
* no commercial use allowed, the full terms of which are made available
* in the LICENSE file. No license in patents is granted.
*
* You can contact the author at <roland.jung@aau.at>
******************************************************************************/
#ifndef IKF_SCHMIDTKALMANFILTERHANDLER_HPP
#define IKF_SCHMIDTKALMANFILTERHANDLER_HPP

#include <ikf/ikf_api.h>
#include <ikf/Estimator/IIsolatedKalmanFilter.hpp>
#include <ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp>
#include <memory>

namespace ikf {

class IKF_API SchmidtKalmanFilterHandler : public IsolatedKalmanFilterHandler {
public:
  SchmidtKalmanFilterHandler(double const horizon_sec = 1.0);
  ~SchmidtKalmanFilterHandler() = default;

  /// Generic fusion algorithm for M-participants:
  /// - the state dim can be infered from the cols of the H matrices
  /// - IDs of particants can be obtained through the dictionary keys.
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
  KalmanFilter::CorrectionResult_t correction_step(const Timestamp &t, Eigen::MatrixXd const& H, Eigen::MatrixXd const& R,
                                            Eigen::VectorXd const & r, std::map<size_t, pBelief_t> &dict_bel,
                                            KalmanFilter::CorrectionCfg_t const& cfg);

  virtual void correct_beliefs_implace(Eigen::MatrixXd &Sigma_apos, Eigen::VectorXd &delta_mean,
                                                            std::map<size_t, pBelief_t> const &dict_bel) override;
};  // class SchmidtKalmanFilterHandler

}  // namespace ikf

#endif // SCHMIDTKALMANFILTERHANDLER_HPP
