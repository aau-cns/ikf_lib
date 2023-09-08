/******************************************************************************
 * FILENAME:     DecoupledPropagationHandler.cpp
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
#include <ikf/EstimatorHandler/DecoupledPropagationHandler.hpp>
#include <ikf/Logger/Logger.hpp>
#include <ikf/utils/eigen_utils.hpp>

namespace ikf {

DecoupledPropagationHandler::DecoupledPropagationHandler(const double horizon_sec)
  : IsolatedKalmanFilterHandler(horizon_sec) {
  Logger::ikf_logger()->info("DecoupledPropagationHandler will perform updates on the local full-state!");
}

Eigen::MatrixXd DecoupledPropagationHandler::stack_H(const std::map<size_t, Eigen::MatrixXd> &dict_H) {
  size_t state_dim = 0;
  size_t H_rows = 0;
  for (auto const &e : dict_H) {
    H_rows = e.second.rows();
  }

  std::map<size_t, size_t> dict_dim;
  std::vector<size_t> IDs = get_instance_ids();
  for (auto const ID : IDs) {
    size_t dim_I = get(ID)->current_belief()->es_dim();
    state_dim += dim_I;
    dict_dim.insert({ID, dim_I});
  }

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(H_rows, state_dim);
  {
    size_t col_start = 0;
    for (auto const &e : dict_dim) {
      size_t dim_I = e.second;
      if (dict_H.find(e.first) != dict_H.end()) {
        H.block(0, col_start, H_rows, dim_I) = dict_H.at(e.first);
      }
      col_start += dim_I;
    }
  }
  return H;
}

std::map<size_t, pBelief_t> DecoupledPropagationHandler::get_dict_bel(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                                      const Timestamp &t) {
  std::map<size_t, pBelief_t> dict_bel;
  std::map<size_t, size_t> dict_dim;
  std::vector<size_t> IDs = get_instance_ids();
  for (auto const id : IDs) {
    pBelief_t bel_apri;
    RTV_EXPECT_TRUE_THROW(get(id)->get_belief_at_t(t, bel_apri, eGetBeliefStrategy::AUTO), "Could not obtain belief");
    dict_bel.insert({id, bel_apri});
  }
  return dict_bel;
}

bool DecoupledPropagationHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                    const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
                                                    const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg) {
  Eigen::MatrixXd H = stack_H(dict_H);

  std::map<size_t, pBelief_t> dict_bel = get_dict_bel(dict_H, t);

  Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);

  // stack individual's covariances:
  Sigma_apri = utils::stabilize_covariance(Sigma_apri);
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());

  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H, R, r, Sigma_apri, cfg);
  if (!res.rejected) {
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(res.Sigma_apos),
                        "Joint apos covariance is not PSD at t=" + t.str());

    // 2) set a corrected factorized a posterioiry cross-covariance
    split_right_upper_covariance(res.Sigma_apos, dict_bel, t);

    // 3) correct beliefs implace!
    correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);
  }
  return !res.rejected;
}

bool DecoupledPropagationHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                    const Eigen::VectorXd &z, const Eigen::MatrixXd &R,
                                                    const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg) {
  Eigen::MatrixXd H = stack_H(dict_H);

  std::map<size_t, pBelief_t> dict_bel = get_dict_bel(dict_H, t);

  Eigen::VectorXd mean_apri = stack_mean(dict_bel);
  Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);

  Eigen::VectorXd r = z - H * mean_apri;

  // stack individual's covariances:
  Sigma_apri = utils::stabilize_covariance(Sigma_apri);
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());

  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H, R, r, Sigma_apri, cfg);
  if (!res.rejected) {
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(res.Sigma_apos),
                        "Joint apos covariance is not PSD at t=" + t.str());

    // 2) set a corrected factorized a posterioiry cross-covariance
    split_right_upper_covariance(res.Sigma_apos, dict_bel, t);

    // 3) correct beliefs implace!
    correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);
  }
  return !res.rejected;
}

}  // namespace ikf
