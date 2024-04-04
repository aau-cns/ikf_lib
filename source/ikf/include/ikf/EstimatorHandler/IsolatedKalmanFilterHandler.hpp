/******************************************************************************
* FILENAME:     IsolatedKalmanFilterHandler.hpp
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
#ifndef ISOLATEDKALMANFILTERHANDLER_HPP
#define ISOLATEDKALMANFILTERHANDLER_HPP
#include <ikf/Estimator/IIsolatedKalmanFilter.hpp>
#include <ikf/EstimatorHandler/IDICOHandler.hpp>
#include <ikf/ikf_api.h>
#include <memory>

namespace ikf {

class IKF_API IsolatedKalmanFilterHandler : public IDICOHandler {
public:
  IsolatedKalmanFilterHandler(double const horizon_sec = 1.0);
  ~IsolatedKalmanFilterHandler() = default;

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
  virtual Eigen::MatrixXd get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, Timestamp const& t);
  virtual void set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd& Sigma_IJ,
                                 const Timestamp& t);

  virtual Eigen::MatrixXd stack_H(const std::map<size_t, Eigen::MatrixXd>& dict_H);

  virtual std::map<size_t, pBelief_t> get_dict_bel(const std::map<size_t, Eigen::MatrixXd>& dict_H, Timestamp const& t);

  virtual std::map<size_t, pBelief_t> get_dict_bel(const std::vector<size_t>& ids, Timestamp const& t);

  virtual size_t get_dim(std::map<size_t, pBelief_t> const& dict_bel);
  Eigen::VectorXd stack_mean(const std::map<size_t, pBelief_t>& dict_bel);
  virtual Eigen::MatrixXd stack_Sigma(const std::map<size_t, pBelief_t>& dict_bel, Timestamp const& t);

  virtual void apply_corrections_at_t(Eigen::MatrixXd& Sigma_apos, const std::map<size_t, pBelief_t>& dict_bel,
                                      Timestamp const& t);

  virtual void split_right_upper_covariance(Eigen::MatrixXd& Sigma, const std::map<size_t, pBelief_t>& dict_bel,
                                            Timestamp const& t);

  void correct_beliefs_implace(Eigen::MatrixXd& Sigma_apos, Eigen::VectorXd& delta_mean,
                               const std::map<size_t, pBelief_t>& dict_bel);

  virtual ApplyObsResult_t process_observation(const Eigen::MatrixXd& R, const Eigen::VectorXd& z, const Timestamp& t,
                                               IIsolatedKalmanFilter::h_joint const& h, std::vector<size_t> const& IDs,
                                               const KalmanFilter::CorrectionCfg_t& cfg, Eigen::MatrixXd& Sigma_apos,
                                               Eigen::VectorXd& dx, std::map<size_t, ikf::pBelief_t>& dict_bel);

};  // class IsolatedKalmanFilterHandler

}  // namespace ikf

#endif // ISOLATEDKALMANFILTERHANDLER_HPP
