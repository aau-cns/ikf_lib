/******************************************************************************
 * FILENAME:     DCI_IKF_Handler.cpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     26.06.2024
 *
 *  Copyright (C) 2024 Roland Jung, Control of Networked Systems, University of Klagenfurt, Austria.
 *
 * All rights reserved.
 *
 * This software is licensed under the terms of the BSD-2-Clause-License with
 * no commercial use allowed, the full terms of which are made available
 * in the LICENSE file. No license in patents is granted.
 *
 * You can contact the author at <roland.jung@aau.at>
 *
 * reference:
 * [a] P. Zhu, Y. Yang, W. Ren and G. Huang, "Cooperative Visual-Inertial Odometry," 2021 IEEE International Conference
 *on Robotics and Automation (ICRA),
 *
 ******************************************************************************/

#include <ikf/EstimatorHandler/DCI_IKF_Handler.hpp>
#include <ikf/Logger/Logger.hpp>
#include <ikf/utils/eigen_utils.hpp>
#include <ikf/utils/lock_guard_timed.hpp>
namespace ikf {

DCI_IKF_Handler::DCI_IKF_Handler(MultiAgentHdl_ptr pAgentHdler, const double horizon_sec)
  : ICSE_IKF_Handler(pAgentHdler, horizon_sec) {
  Logger::ikf_logger()->info(
    "DCI_IKF_Handler(): will perform inter-agent observation decoupled using a CI-EKF update!");
}

ApplyObsResult_t DCI_IKF_Handler::apply_inter_agent_observation(
  const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t, const IIsolatedKalmanFilter::h_joint &h,
  const std::vector<size_t> &IDs, const KalmanFilter::CorrectionCfg_t &cfg,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &remote_IDs,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &local_IDs) {
  //
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> remote_beliefs, local_beliefs;
    std::map<size_t, std::map<size_t, Eigen::MatrixXd>> remote_FFCs, local_FFCs;
    if (!get_local_beliefs_and_FCC_at_t(local_IDs, local_IDs, t, local_beliefs, local_FFCs)) {
      ikf::Logger::ikf_logger()->error("CI_IKF_Handler::apply_inter_agent_observation: failed to obtain local data...");
      return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
    }

    if (local_beliefs.empty()) {
      return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
    }

    if (!m_pAgentHandler->get_beliefs_and_FCC_at_t(remote_IDs, remote_IDs, t, remote_beliefs, remote_FFCs)) {
      ikf::Logger::ikf_logger()->error(
        "DCI_IKF_Handler::apply_inter_agent_observation: failed to obtain remote data...");
      return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
    }
    if (remote_beliefs.empty()) {
      return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
    }

    ikf::Logger::ikf_logger()->debug("DCI_IKF_Handler::apply_inter_agent_observation() stack beliefs...");

    // STACK BELIEFS AND Factorized-Cross-Covariances (FFCs)
    std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> dict_bel;
    dict_bel.insert(local_beliefs.begin(), local_beliefs.end());
    dict_bel.insert(remote_beliefs.begin(), remote_beliefs.end());

    // linearize the measurement function with the beliefs:
    std::pair<std::map<size_t, Eigen::MatrixXd>, Eigen::VectorXd> H_r = h(dict_bel, IDs, z);

    // split up H into H_local, H_remote
    // split up Sigma into Sigma_local, Sigma_remote
    std::map<size_t, Eigen::MatrixXd> dict_H_ii, dict_H_jj;
    for (auto id_i : local_IDs) {
      if (H_r.first.find(id_i) != H_r.first.end()) {
        dict_H_ii[id_i] = H_r.first.at(id_i);
      }
    }
    for (auto id_i : remote_IDs) {
      if (H_r.first.find(id_i) != H_r.first.end()) {
        dict_H_jj[id_i] = H_r.first.at(id_i);
      }
    }
    // remove fixed measurement Jacobians, beliefs, and FCCs again:
    for (auto iter_bel_I = local_beliefs.cbegin(); iter_bel_I != local_beliefs.cend();) {
      if (iter_bel_I->second->options().is_fixed) {
        dict_H_ii.erase(iter_bel_I->first);   // remove a column of H
        local_FFCs.erase(iter_bel_I->first);  // remove a row of FFCs
        iter_bel_I = local_beliefs.erase(iter_bel_I);
      } else {
        ++iter_bel_I;
      }
    }
    // remove fixed measurement Jacobians, beliefs, and FCCs again:
    for (auto iter_bel_I = remote_beliefs.cbegin(); iter_bel_I != remote_beliefs.cend();) {
      if (iter_bel_I->second->options().is_fixed) {
        dict_H_jj.erase(iter_bel_I->first);    // remove a column of H
        remote_FFCs.erase(iter_bel_I->first);  // remove a row of FFCs
        iter_bel_I = remote_beliefs.erase(iter_bel_I);
      } else {
        ++iter_bel_I;
      }
    }

    Eigen::MatrixXd H_ii = stack_H(dict_H_ii);
    Eigen::MatrixXd H_jj = stack_H(dict_H_jj);

    Eigen::MatrixXd Sigma_ii_apri = stack_Sigma_locally(local_beliefs, t, local_FFCs);
    Eigen::MatrixXd Sigma_jj_apri = stack_Sigma_locally(remote_beliefs, t, remote_FFCs);

    // stack individual's covariances:
    RTV_EXPECT_TRUE_MSG(utils::correct_covariance(Sigma_ii_apri),
                        "DCI_IKF_Handler::apply_inter_agent_observation(): apri Sigma_II is not PSD at t=" + t.str());
    RTV_EXPECT_TRUE_MSG(utils::correct_covariance(Sigma_jj_apri),
                        "DCI_IKF_Handler::apply_inter_agent_observation(): apri Sigma_JJ is not PSD at t=" + t.str());

    // constant: Sec 5 [a]
    double omega_i = 0.995;
    auto res = KalmanFilter::covariance_intersection_correction(H_ii, H_jj, R, H_r.second, Sigma_ii_apri, Sigma_jj_apri,
                                                                omega_i, cfg);
    if (!res.rejected) {
      bool is_psd = utils::is_positive_semidefinite(res.Sigma_apos);
      RTV_EXPECT_TRUE_MSG(is_psd, "Joint apos covariance is not PSD at t=" + t.str());
      if (!is_psd) {
        res.Sigma_apos = utils::nearest_covariance(res.Sigma_apos, 1e-6);
      }
      Eigen::MatrixXd Sigma_ii_apos = res.Sigma_apos;
      Eigen::VectorXd delta_x = res.delta_mean;

      // IMPORTANT: MAINTAIN ORDER STRICKTLY
      // 1) LOCAL: add correction terms on all a aprior factorized cross-covariances!
      ikf::Logger::ikf_logger()->debug("DCI_IKF_Handler::apply_inter_agent_observation(): apply_corrections_at_t...");
      apply_corrections_at_t(Sigma_ii_apos, local_beliefs, t);

      // 2) LOCAL: afterwards, overwrite/set factorized a posterioiry cross-cov (apply no corrections afterwards on
      split_right_upper_covariance(Sigma_ii_apos, local_beliefs, t);

      // 3) LOCAL: correct beliefs implace!
      correct_beliefs_implace(Sigma_ii_apos, delta_x, local_beliefs);

      ikf::Logger::ikf_logger()->debug("DCI_IKF_Handler::apply_inter_agent_observation(): DONE!");
      return ApplyObsResult_t(eMeasStatus::PROCESSED, H_r.second);
    }
    return ApplyObsResult_t(eMeasStatus::REJECTED, H_r.second);
  } else {
    ikf::Logger::ikf_logger()->error("DCI_IKF_Handler::apply_inter_agent_observation(): mutex FAILED");
    return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
  }
}

}  // namespace ikf
