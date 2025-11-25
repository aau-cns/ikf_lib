/******************************************************************************
 * FILENAME:     SchmidtKalmanFilterHandler.cpp
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
#include <ikf/Estimator/KalmanFilter.hpp>
#include <ikf/Estimator/NormalizedInnovationSquared.hpp>
#include <ikf/EstimatorHandler/SchmidtKalmanFilterHandler.hpp>
#include <ikf/Logger/Logger.hpp>
#include <ikf/utils/eigen_utils.hpp>
#include <ikf/utils/lock_guard_timed.hpp>

namespace ikf {

SchmidtKalmanFilterHandler::SchmidtKalmanFilterHandler(const double horizon_sec) : IsolatedKalmanFilterHandler(horizon_sec) {
  Logger::ikf_logger()->info("SchmidtKalmanFilterHandler: nuissance parameter won't be corrected.");
}

bool SchmidtKalmanFilterHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                    const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
                                                    const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    Eigen::MatrixXd H = stack_H(dict_H);

    std::map<size_t, pBelief_t> dict_bel = get_dict_bel(dict_H, t);
    if (dict_bel.empty()) {
      return false;
    }

    Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);

    // stack individual's covariances:
    //Sigma_apri = utils::stabilize_covariance(Sigma_apri);
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri),
                        "Joint apri covariance is not PSD at t=" + t.str());

    KalmanFilter::CorrectionResult_t res;
    // here we need a SKF update, distingishing essential and nuissance parameters: dx for essential must be 0, thus K_nuissance = 0.
    //res = KalmanFilter::correction_step(H, R, r, Sigma_apri, cfg);
    res = correction_step(t, H, R, r, dict_bel, cfg);
    if (!res.rejected) {
      bool is_psd = utils::is_positive_semidefinite(res.Sigma_apos);
      RTV_EXPECT_TRUE_MSG(is_psd, "Joint apos covariance is not PSD at t=" + t.str());
      if (!is_psd) {
        res.Sigma_apos = utils::nearest_covariance(res.Sigma_apos, 1e-6);
      }

      // IMPORTANT: MAINTAIN ORDER STRICKTLY
      // 1) add correction terms on all a aprior factorized cross-covariances!
      // this should only operate on the essential parameters. Between nuissance and others should be no change.
      apply_corrections_at_t(res.Sigma_apos, dict_bel, t);

      // 2) afterwards, overwrite/set factorized a posterioiry cross-covariance (apply no corrections afterwards on
      // them)
      split_right_upper_covariance(res.Sigma_apos, dict_bel, t);

      // 3) correct beliefs implace!
      correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);
    }
    return !res.rejected;
  } else {
    ikf::Logger::ikf_logger()->error("IsolatedKalmanFilterHandler::apply_observation(): mutex FAILED");
    return false;
  }
}

bool SchmidtKalmanFilterHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                    const Eigen::VectorXd &z, const Eigen::MatrixXd &R,
                                                    const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    Eigen::MatrixXd H = stack_H(dict_H);

    std::map<size_t, pBelief_t> dict_bel = get_dict_bel(dict_H, t);
    if (dict_bel.empty()) {
      return false;
    }
    Eigen::VectorXd mean_apri = stack_mean(dict_bel);
    Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);

    Eigen::VectorXd r = z - H * mean_apri;

    // stack individual's covariances:<
    Sigma_apri = utils::stabilize_covariance(Sigma_apri);
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri),
                        "Joint apri covariance is not PSD at t=" + t.str());

    KalmanFilter::CorrectionResult_t res;
    res = correction_step(t, H, R, r, dict_bel, cfg);
    if (!res.rejected) {
      bool is_psd = utils::is_positive_semidefinite(res.Sigma_apos);
      RTV_EXPECT_TRUE_MSG(is_psd, "Joint apos covariance is not PSD at t=" + t.str());
      if (!is_psd) {
        res.Sigma_apos = utils::nearest_covariance(res.Sigma_apos, 1e-6);
      }

      // IMPORTANT: MAINTAIN ORDER STRICKTLY
      // 1) add correction terms in the appropriate correction buffers!
      apply_corrections_at_t(res.Sigma_apos, dict_bel, t);

      // 2) set a corrected factorized a posterioiry cross-covariance
      split_right_upper_covariance(res.Sigma_apos, dict_bel, t);

      // 3) correct beliefs implace!
      correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);
    }
    return !res.rejected;
  } else {
    ikf::Logger::ikf_logger()->error("IsolatedKalmanFilterHandler::apply_observation(): mutex FAILED");
    return false;
  }
}

ApplyObsResult_t SchmidtKalmanFilterHandler::apply_observation(const Eigen::MatrixXd &R, const Eigen::VectorXd &z,
                                                                const Timestamp &t,
                                                                IIsolatedKalmanFilter::h_joint const &h,
                                                                const std::vector<size_t> &IDs,
                                                                const KalmanFilter::CorrectionCfg_t &cfg) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    std::map<size_t, pBelief_t> dict_bel = get_dict_bel(IDs, t);
    if (dict_bel.empty()) {
      return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
    }
    // only unfixed means and Sigmas are stacked
    Eigen::VectorXd mean_apri = stack_mean(dict_bel);
    Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);

    std::pair<std::map<size_t, Eigen::MatrixXd>, Eigen::VectorXd> H_r = h(dict_bel, IDs, z);

    // remove fixed measurement Jacobians again:
    for (auto &bel_I : dict_bel) {
      if (bel_I.second->options().is_fixed) {
        H_r.first.erase(bel_I.first);
      }
    }

    // stack Jacobian
    Eigen::MatrixXd H = stack_H(H_r.first);

    // compute residual
    Eigen::VectorXd r = H_r.second;


    // stack individual's covariances:<
    Sigma_apri = utils::stabilize_covariance(Sigma_apri);
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri),
                        "Joint apri covariance is not PSD at t=" + t.str());

    KalmanFilter::CorrectionResult_t res;
    res = correction_step(t, H, R, r, dict_bel, cfg);
    if (!res.rejected) {
      bool is_psd = utils::is_positive_semidefinite(res.Sigma_apos);
      RTV_EXPECT_TRUE_MSG(is_psd, "Joint apos covariance is not PSD at t=" + t.str());
      if (!is_psd) {
        res.Sigma_apos = utils::nearest_covariance(res.Sigma_apos, 1e-6);
      }

      // IMPORTANT: MAINTAIN ORDER STRICKTLY
      // 1) add correction terms in the appropriate correction buffers!
      apply_corrections_at_t(res.Sigma_apos, dict_bel, t);

      // 2) set a corrected factorized a posterioiry cross-covariance
      split_right_upper_covariance(res.Sigma_apos, dict_bel, t);

      // 3) correct beliefs implace!
      correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);
    }
    return ApplyObsResult_t(eMeasStatus::PROCESSED);
  } else {
    ikf::Logger::ikf_logger()->error("IsolatedKalmanFilterHandler::apply_observation(): mutex FAILED");
    return ApplyObsResult_t(eMeasStatus::REJECTED);
  }
}



KalmanFilter::CorrectionResult_t SchmidtKalmanFilterHandler::correction_step(const Timestamp &t, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, const Eigen::VectorXd &r, std::map<size_t, pBelief_t>& dict_bel, const KalmanFilter::CorrectionCfg_t &cfg) {
  KalmanFilter::CorrectionResult_t res;

  Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);
  if (KalmanFilter::check_dim(H, R, r, Sigma_apri)) {
    Eigen::Index const dim = Sigma_apri.rows();
    res.rejected = false;

    // innovation covariance:
    Eigen::MatrixXd S = H * Sigma_apri  * H.transpose() + R;
    S = utils::stabilize_covariance(S, cfg.eps);
    if (cfg.use_outlier_rejection) {
      if (!NormalizedInnovationSquared::check_NIS(S, r, cfg.confidence_interval)) {
        res.rejected = true;
      }
    }

    // The optimal Kalman gain:
    Eigen::MatrixXd K = Sigma_apri*H.transpose() * S.inverse();
    // The (suboptimal) Schmidt Kalman gain
    Eigen::MatrixXd K_SKF = Eigen::MatrixXd::Zero(K.rows(), K.cols());

    // Set the Kalman gain for all nuissance parameters to 0!
    Eigen::Index row_start = 0;
    for (auto &bel_I : dict_bel) {
      Eigen::Index state_dim_row = bel_I.second->ns_dim();
      if (bel_I.second->options().is_essential) {
        K_SKF.block(row_start, 0, state_dim_row , H.rows()) = K.block(row_start, 0, state_dim_row , H.rows());
      }
      row_start += state_dim_row;
    }

    res.delta_mean = K_SKF * r;
    res.U = (Eigen::MatrixXd::Identity(dim, dim) - K_SKF*H);

    // use Joseph's form
    res.Sigma_apos =  res.U * Sigma_apri * res.U.transpose() + K_SKF * R * K_SKF.transpose();

    //if (cfg.nummerical_stabilization) {
    res.Sigma_apos = utils::symmetrize_covariance(res.Sigma_apos);
    //}
  }
  return res;
}

void SchmidtKalmanFilterHandler::correct_beliefs_implace(Eigen::MatrixXd &Sigma_apos, Eigen::VectorXd &delta_mean,
                                                          std::map<size_t, pBelief_t> const &dict_bel) {
  size_t row_start = 0;
  for (auto const &e_i : dict_bel) {
    size_t dim_I = e_i.second->es_dim();
    if (e_i.second->options().is_essential) {
      Eigen::MatrixXd Sigma_II_apos = Sigma_apos.block(row_start, row_start, dim_I, dim_I);
      e_i.second->correct(delta_mean.block(row_start, 0, dim_I, 1), Sigma_II_apos);
    }
    row_start += dim_I;
  }
}


} // ns ikf
