/******************************************************************************
 * FILENAME:     CollaborativeIKFHandler.cpp
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
#include <ikf/EstimatorHandler/CollaborativeIKFHandler.hpp>
#include <ikf/Logger/Logger.hpp>
#include <ikf/utils/eigen_utils.hpp>
#include <ikf/utils/lock_guard_timed.hpp>

namespace ikf {

std::string to_string(const eRedoUpdateStrategy t) {
  switch (t) {
  case eRedoUpdateStrategy::EXACT:
    return "EXACT";
  case eRedoUpdateStrategy::POSTCORRELATED:
    return "POSTCORRELATED";
  case eRedoUpdateStrategy::DISCARD:
    return "DISCARD";
  default:
    break;
  }
  return std::string();
}

CollaborativeIKFHandler::CollaborativeIKFHandler(MultiAgentHdl_ptr pAgentHdler, const double horizon_sec)
  : ICSE_IKF_Handler(pAgentHdler, horizon_sec), mRedoStrategy(eRedoUpdateStrategy::POSTCORRELATED) {
  Logger::ikf_logger()->info(
    "CollaborativeIKFHandler(): will perform inter-agent observation isolated (partially coupled) using IKF update!");
  Logger::ikf_logger()->info("* RedoStrategy=" + to_string(mRedoStrategy));
}

std::map<size_t, pBelief_t> CollaborativeIKFHandler::get_dict_bel(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                                  const Timestamp &t) {
  std::vector<size_t> IDs;
  IDs.reserve(dict_H.size());
  for (auto const &e : dict_H) {
    IDs.push_back(e.first);
  }
  return ICSE_IKF_Handler::get_dict_bel(IDs, t);
}

Eigen::MatrixXd CollaborativeIKFHandler::get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Timestamp &t) {
  Eigen::MatrixXd SigmaFact_IJ, SigmaFact_JI;
  if (exists(ID_I)) {
    SigmaFact_IJ = get(ID_I)->get_CrossCovFact_at_t(t, ID_J);
  } else {
    SigmaFact_IJ = m_pAgentHandler->get_CrossCovFact_IJ_at_t(ID_I, ID_J, t);
  }
  if (exists(ID_J)) {
    SigmaFact_JI = get(ID_J)->get_CrossCovFact_at_t(t, ID_I);
  } else {
    SigmaFact_JI = m_pAgentHandler->get_CrossCovFact_IJ_at_t(ID_J, ID_I, t);
  }
  Eigen::MatrixXd Sigma_IJ;
  if (SigmaFact_IJ.size() && SigmaFact_JI.size()) {
    if (ID_I < ID_J) {
      RTV_EXPECT_TRUE_MSG(SigmaFact_IJ.cols() == SigmaFact_JI.cols(), "get_crosscov wrong dims @t=" + t.str());
      Sigma_IJ = SigmaFact_IJ * SigmaFact_JI.transpose();

    } else {
      RTV_EXPECT_TRUE_MSG(SigmaFact_IJ.cols() == SigmaFact_JI.cols(), "get_crosscov wrong dims @t=" + t.str());
      Eigen::MatrixXd Sigma_JI = SigmaFact_JI * SigmaFact_IJ.transpose();
      Sigma_IJ = Sigma_JI.transpose();
    }
  }
  return Sigma_IJ;
}

//  Eq (1) and Algorithm 6 in [1]
void CollaborativeIKFHandler::set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &Sigma_IJ,
                                                const Timestamp &t) {
  if (ID_I < ID_J) {
    if (exists(ID_I)) {
      get(ID_I)->set_CrossCovFact_at_t(t, ID_J, Sigma_IJ);
    } else {
      m_pAgentHandler->set_CrossCovFact_IJ_at_t(ID_I, ID_J, Sigma_IJ, t);
    }
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Sigma_IJ.cols(), Sigma_IJ.cols());
    if (exists(ID_J)) {
      get(ID_J)->set_CrossCovFact_at_t(t, ID_I, I);
    } else {
      m_pAgentHandler->set_CrossCovFact_IJ_at_t(ID_J, ID_I, I, t);
    }
  } else {
    Eigen::MatrixXd Sigma_JI = Sigma_IJ.transpose();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Sigma_JI.cols(), Sigma_JI.cols());
    if (exists(ID_J)) {
      get(ID_J)->set_CrossCovFact_at_t(t, ID_I, Sigma_JI);
    } else {
      m_pAgentHandler->set_CrossCovFact_IJ_at_t(ID_J, ID_I, Sigma_JI, t);
    }

    if (exists(ID_I)) {
      get(ID_I)->set_CrossCovFact_at_t(t, ID_J, I);
    } else {
      m_pAgentHandler->set_CrossCovFact_IJ_at_t(ID_I, ID_J, I, t);
    }
  }
}

void CollaborativeIKFHandler::apply_corrections_at_t(Eigen::MatrixXd &Sigma_apos,
                                                     const std::map<size_t, pBelief_t> &dict_bel, const Timestamp &t,
                                                     bool const only_local_beliefs) {
  size_t row_start = 0;
  for (auto const &e_i : dict_bel) {
    size_t dim_I = e_i.second->es_dim();

    Eigen::MatrixXd Sigma_II_apos = Sigma_apos.block(row_start, row_start, dim_I, dim_I);
    if (exists(e_i.first)) {
      get(e_i.first)->apply_correction_at_t(t, e_i.second->Sigma(), Sigma_II_apos);
    } else if (!only_local_beliefs) {
      m_pAgentHandler->apply_correction_at_t(e_i.first, t, e_i.second->Sigma(), Sigma_II_apos);
    }
    row_start += dim_I;
  }
}

std::set<size_t> CollaborativeIKFHandler::get_correlated_IDs_after_t(const Timestamp &t) {
  std::set<size_t> IDs_post_corr;
  for (auto ID : this->get_instance_ids()) {
    auto IDs = this->get(ID)->get_correlated_IDs_after_t(t);
    IDs_post_corr.insert(IDs.begin(), IDs.end());
  }
  return IDs_post_corr;
}

std::set<size_t> CollaborativeIKFHandler::get_remote_correlated_IDs_after_t(const Timestamp &t) {
  std::set<size_t> IDs_post_corr_remote;
  for (auto ID : get_correlated_IDs_after_t(t)) {
    if (!exists(ID)) {
      IDs_post_corr_remote.insert(ID);
    }
  }
  return IDs_post_corr_remote;
}

bool CollaborativeIKFHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t,
                                                const KalmanFilter::CorrectionCfg_t &cfg) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    std::set<IMultiAgentHandler::IDAgent_t> ID_agents;
    std::vector<IMultiAgentHandler::IDEstimator_t> remote_IDs;
    std::vector<IMultiAgentHandler::IDEstimator_t> local_IDs;

    for (auto const &e : dict_H) {
      if (!exists(e.first)) {
        ID_agents.insert(m_pAgentHandler->estimatorID2agentID(e.first));
        remote_IDs.push_back(e.first);
      } else {
        local_IDs.push_back(e.first);
      }
    }

    ikf::Logger::ikf_logger()->info("CollaborativeIKFHandler::apply_observation(): num remote IDs="
                                    + std::to_string(remote_IDs.size())
                                    + ", num ID_agents=" + std::to_string(ID_agents.size()));

    // only local instances involved... we are done here!
    if (remote_IDs.size() == 0) {
      return IsolatedKalmanFilterHandler::apply_observation(dict_H, R, r, t, cfg);
    }

    if (ID_agents.size() == 1) {
      // we can perform an optimized update using jumbo messages!

      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::apply_corrections_at_t(): apply_inter_agent_observation...");

      return apply_inter_agent_observation(dict_H, R, r, t, cfg, remote_IDs, local_IDs);
    }

    Eigen::MatrixXd H = stack_H(dict_H);

    std::map<size_t, pBelief_t> dict_bel = get_dict_bel(dict_H, t);
    if (dict_bel.empty()) {
      return false;
    }

    Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);

    // stack individual's covariances:
    Sigma_apri = utils::stabilize_covariance(Sigma_apri);
    bool is_psd = utils::is_positive_semidefinite(Sigma_apri);
    RTV_EXPECT_TRUE_MSG(is_psd, "Joint apri covariance is not PSD at t=" + t.str());
    if (!is_psd) {
      Sigma_apri = utils::nearest_covariance(Sigma_apri, 1e-6);
    }

    KalmanFilter::CorrectionResult_t res;
    res = KalmanFilter::correction_step(H, R, r, Sigma_apri, cfg);
    if (!res.rejected) {
      bool is_psd = utils::is_positive_semidefinite(res.Sigma_apos);
      RTV_EXPECT_TRUE_MSG(is_psd, "Joint apos covariance is not PSD at t=" + t.str());
      if (!is_psd) {
        res.Sigma_apos = utils::nearest_covariance(res.Sigma_apos, 1e-6);
      }

      // IMPORTANT: MAINTAIN ORDER STRICKTLY
      // 1) add correction terms on all a aprior factorized cross-covariances!
      ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_corrections_at_t():...");
      apply_corrections_at_t(res.Sigma_apos, dict_bel, t, false);

      // 2) afterwards, overwrite/set factorized a posterioiry cross-covariance (apply no corrections afterwards on
      // them)
      ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::split_right_upper_covariance() ...");
      split_right_upper_covariance(res.Sigma_apos, dict_bel, t);

      // 3) correct beliefs implace!

      ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::correct_beliefs_implace():...");
      correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);

      // 4) send corrected beliefs back
      ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_observation(): send corrected beliefs back ...");
      for (auto const &id : remote_IDs) {
        m_pAgentHandler->set_belief_at_t(id, t, dict_bel.at(id));
      }

      // 5) redo updates after t
      ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_observation():  redo updates after t ...");
      for (auto const &id : ID_agents) {
        if (!m_pAgentHandler->redo_updates_after_t(id, t)) {
          ikf::Logger::ikf_logger()->warn(
            "CollaborativeIKFHandler::apply_observation(): redo_update(agent={:d}) failed", id);
        }
      }
    }
    return !res.rejected;
  } else {
    ikf::Logger::ikf_logger()->error("CollaborativeIKFHandler::apply_observation(): mutex FAILED");
    return false;
  }
}

bool CollaborativeIKFHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                const Eigen::VectorXd &z, const Eigen::MatrixXd &R, const Timestamp &t,
                                                const KalmanFilter::CorrectionCfg_t &cfg) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    std::vector<IMultiAgentHandler::IDEstimator_t> ID_remote_participants;
    for (auto const &e : dict_H) {
      if (!exists(e.first)) {
        ID_remote_participants.push_back(e.first);
      }
    }

    // only local instances involved... we are done here!
    if (ID_remote_participants.size() == 0) {
      return IsolatedKalmanFilterHandler::apply_observation(dict_H, z, R, t, cfg);
    }

    Eigen::MatrixXd H = stack_H(dict_H);

    std::map<size_t, pBelief_t> dict_bel = get_dict_bel(dict_H, t);
    if (dict_bel.empty()) {
      return false;
    }

    Eigen::VectorXd mean_apri = stack_mean(dict_bel);
    Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);

    Eigen::VectorXd r = z - H * mean_apri;

    // stack individual's covariances:
    Sigma_apri = utils::stabilize_covariance(Sigma_apri);
    bool is_psd = utils::is_positive_semidefinite(Sigma_apri);
    RTV_EXPECT_TRUE_MSG(is_psd, "Joint apri covariance is not PSD at t=" + t.str());
    if (!is_psd) {
      Sigma_apri = utils::nearest_covariance(Sigma_apri, 1e-6);
    }

    KalmanFilter::CorrectionResult_t res;
    res = KalmanFilter::correction_step(H, R, r, Sigma_apri, cfg);
    if (!res.rejected) {
      bool is_psd = utils::is_positive_semidefinite(res.Sigma_apos);
      RTV_EXPECT_TRUE_MSG(is_psd, "Joint apos covariance is not PSD at t=" + t.str());
      if (!is_psd) {
        res.Sigma_apos = utils::nearest_covariance(res.Sigma_apos, 1e-6);
      }

      // IMPORTANT: MAINTAIN ORDER STRICKTLY
      // 1) add correction terms on all a aprior factorized cross-covariances!
      apply_corrections_at_t(res.Sigma_apos, dict_bel, t, false);

      // 2) afterwards, overwrite/set factorized a posterioiry cross-covariance (apply no corrections afterwards on
      // them)
      split_right_upper_covariance(res.Sigma_apos, dict_bel, t);

      // 3) correct beliefs implace!
      correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);

      // 4) send corrected beliefs back
      for (auto const &id : ID_remote_participants) {
        m_pAgentHandler->set_belief_at_t(id, t, dict_bel.at(id));
      }
    }
    return !res.rejected;
  } else {
    ikf::Logger::ikf_logger()->error("CollaborativeIKFHandler::apply_observation(): mutex FAILED");
    return false;
  }
}

ProcessMeasResult_vec_t CollaborativeIKFHandler::redo_updates_after_t(const Timestamp &t) {
  // trigger remote agent asynchonrously to redo updates as well...
  // -> hopefully, we are done before remote ones need our beliefs again...
  if (mRedoStrategy == eRedoUpdateStrategy::POSTCORRELATED) {
    auto ID_agents = IDs_to_Agent_IDs(get_remote_correlated_IDs_after_t(t));
    for (auto const &id : ID_agents) {
      if (!m_pAgentHandler->redo_updates_after_t(id, t)) {
        ikf::Logger::ikf_logger()->warn("CollaborativeIKFHandler::redo_updates_after_t(agent={:d}): FAILED", id);
      }
    }
  }
  if (mRedoStrategy == eRedoUpdateStrategy::EXACT) {
    ikf::Logger::ikf_logger()->error("CollaborativeIKFHandler::redo_updates_after_t(): EXACT NOT IMPLEMENTED!");
  }
  return IDICOHandler::redo_updates_after_t(t);
}

bool CollaborativeIKFHandler::discard_measurement(const MeasData &m) {
  if (mRedoStrategy == eRedoUpdateStrategy::DISCARD) {
    // check if there are remote post-correlated instances after m.t_m, if so, order is NOT violated, and measurement
    // has no effect. Reason: we are not triggering the other agent to redo it's updates after our delayed
    // measurments, we just ignore our delayed measurement in that case.
    std::set<size_t> IDs = get_remote_correlated_IDs_after_t(m.t_m);
    if (!IDs.empty()) {
      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::discard_measurement(): found post-correlated estimators num="
        + std::to_string(IDs.size()));
      // remote ID is post-correlated
      return true;
    }
  }
  return false;
}

ProcessMeasResult_vec_t CollaborativeIKFHandler::redo_updates_from_t(const Timestamp &t) {
  // trigger remote agent asynchonrously to redo updates as well...
  // -> hopefully, we are done before remote ones need our beliefs again...
  if (mRedoStrategy == eRedoUpdateStrategy::POSTCORRELATED) {
    auto ID_agents = IDs_to_Agent_IDs(get_remote_correlated_IDs_after_t(t));
    for (auto const &id : ID_agents) {
      if (!m_pAgentHandler->redo_updates_from_t(id, t)) {
        ikf::Logger::ikf_logger()->warn("CollaborativeIKFHandler::redo_updates_from_t(agent={:d}): FAILED", id);
      }
    }
  }
  if (mRedoStrategy == eRedoUpdateStrategy::EXACT) {
    ikf::Logger::ikf_logger()->error("CollaborativeIKFHandler::redo_updates_from_t(): EXACT NOT IMPLEMENTED!");
  }
  return IDICOHandler::redo_updates_from_t(t);
}

bool CollaborativeIKFHandler::apply_inter_agent_observation(
  const std::map<size_t, Eigen::MatrixXd> &dict_H, const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
  const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &remote_IDs,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &local_IDs) {
  //
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    std::vector<IMultiAgentHandler::IDEstimator_t> ID_participants = remote_IDs;
    ID_participants.insert(ID_participants.end(), local_IDs.begin(), local_IDs.end());

    Eigen::MatrixXd H = stack_H(dict_H);
    std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> remote_beliefs, local_beliefs;
    std::map<size_t, std::map<size_t, Eigen::MatrixXd>> remote_FFCs, local_FFCs;
    if (!get_local_beliefs_and_FCC_at_t(local_IDs, ID_participants, t, local_beliefs, local_FFCs)) {
      ikf::Logger::ikf_logger()->error(
        "CollaborativeIKFHandler::apply_inter_agent_observation(H): failed to obtain local data...");
      return false;
    }
    if (local_beliefs.empty()) {
      return false;
    }
    if (!m_pAgentHandler->get_beliefs_and_FCC_at_t(remote_IDs, ID_participants, t, remote_beliefs, remote_FFCs)) {
      ikf::Logger::ikf_logger()->error(
        "CollaborativeIKFHandler::apply_inter_agent_observation: failed to obtain remote data...");
      return false;
    }
    if (remote_beliefs.empty()) {
      return false;
    }

    ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(H) stack beliefs...");

    // STACK BELIEFS AND Factorized-Cross-Covariances (FFCs)
    std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> dict_bel;
    dict_bel.insert(local_beliefs.begin(), local_beliefs.end());
    dict_bel.insert(remote_beliefs.begin(), remote_beliefs.end());
    std::map<size_t, std::map<size_t, Eigen::MatrixXd>> FCCs;
    FCCs.insert(local_FFCs.begin(), local_FFCs.end());
    FCCs.insert(remote_FFCs.begin(), remote_FFCs.end());

    Eigen::MatrixXd Sigma_apri = stack_Sigma_locally(dict_bel, t, FCCs);

    // stack individual's covariances:
    Sigma_apri = utils::stabilize_covariance(Sigma_apri);
    bool is_psd = utils::is_positive_semidefinite(Sigma_apri);
    RTV_EXPECT_TRUE_MSG(is_psd, "Joint apri covariance is not PSD at t=" + t.str());
    if (!is_psd) {
      Sigma_apri = utils::nearest_covariance(Sigma_apri, 1e-6);
    }

    KalmanFilter::CorrectionResult_t res;
    ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(H): correction step");
    res = KalmanFilter::correction_step(H, R, r, Sigma_apri, cfg);
    if (!res.rejected) {
      bool is_psd = utils::is_positive_semidefinite(res.Sigma_apos);
      RTV_EXPECT_TRUE_MSG(is_psd, "Joint apos covariance is not PSD at t=" + t.str());
      if (!is_psd) {
        res.Sigma_apos = utils::nearest_covariance(res.Sigma_apos, 1e-6);
      }

      // IMPORTANT: MAINTAIN ORDER STRICKTLY
      // 1) LOCAL: add correction terms on all a aprior factorized cross-covariances!
      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::apply_inter_agent_observation(H): apply_corrections_at_t...");
      apply_corrections_at_t(res.Sigma_apos, dict_bel, t, true);

      // 2) LOCAL: afterwards, overwrite/set factorized a posterioiry cross-covariance (apply no corrections afterwards
      // on them)

      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::apply_inter_agent_observation(H): split_Sigma_locally...");

      split_Sigma_locally(res.Sigma_apos, dict_bel, FCCs);

      for (auto const &ID_I : local_IDs) {
        for (auto const &e : FCCs.at(ID_I)) {
          size_t const ID_J = e.first;
          get(ID_I)->set_CrossCovFact_at_t(t, ID_J, e.second);
        }
      }

      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::apply_inter_agent_observation(H): correct_beliefs_implace...");

      // 3) correct beliefs implace! will change local_beliefs and remote_beliefs as well!
      correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);

      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::apply_inter_agent_observation(H): send info to other agent...");

      // 4) send info to other agent:
      remote_FFCs.clear();
      for (auto const &ID_I : remote_IDs) {
        remote_FFCs[ID_I] = FCCs[ID_I];
      }

      // schedules redo updates automatically
      if (!m_pAgentHandler->set_beliefs_and_FCC_at_t(t, remote_beliefs, remote_FFCs, true, true)) {
        ikf::Logger::ikf_logger()->error(
          "CollaborativeIKFHandler::apply_inter_agent_observation: failed set belief on remote agent...");
        return false;
      } else {
        ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(H): DONE!");
        return true;
      }
    }

    return true;
  } else {
    ikf::Logger::ikf_logger()->error("CollaborativeIKFHandler::apply_inter_agent_observation(H): mutex FAILED");
    return false;
  }
}

ApplyObsResult_t CollaborativeIKFHandler::apply_inter_agent_observation(
  const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t, const IIsolatedKalmanFilter::h_joint &h,
  const std::vector<size_t> &IDs, const KalmanFilter::CorrectionCfg_t &cfg,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &remote_IDs,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &local_IDs) {
  //
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    std::vector<IMultiAgentHandler::IDEstimator_t> ID_participants = remote_IDs;
    ID_participants.insert(ID_participants.end(), local_IDs.begin(), local_IDs.end());

    std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> remote_beliefs, local_beliefs;
    std::map<size_t, std::map<size_t, Eigen::MatrixXd>> remote_FFCs, local_FFCs;
    if (!get_local_beliefs_and_FCC_at_t(local_IDs, ID_participants, t, local_beliefs, local_FFCs)) {
      ikf::Logger::ikf_logger()->error(
        "CollaborativeIKFHandler::apply_inter_agent_observation(h()): failed to obtain local data...");
      return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
    }

    if (local_beliefs.empty()) {
      return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
    }

    if (!m_pAgentHandler->get_beliefs_and_FCC_at_t(remote_IDs, ID_participants, t, remote_beliefs, remote_FFCs)) {
      ikf::Logger::ikf_logger()->error(
        "CollaborativeIKFHandler::apply_inter_agent_observation(h()): failed to obtain remote data...");
      return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
    }
    if (remote_beliefs.empty()) {
      return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
    }

    ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(h()) stack beliefs...");

    // STACK BELIEFS AND Factorized-Cross-Covariances (FFCs)
    std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> dict_bel;
    dict_bel.insert(local_beliefs.begin(), local_beliefs.end());
    dict_bel.insert(remote_beliefs.begin(), remote_beliefs.end());
    std::map<size_t, std::map<size_t, Eigen::MatrixXd>> FCCs;
    FCCs.insert(local_FFCs.begin(), local_FFCs.end());
    FCCs.insert(remote_FFCs.begin(), remote_FFCs.end());

    for (auto ID : IDs) {
      if (dict_bel.find(ID) == dict_bel.end()) {
        ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(h()): belief for ID="
                                         + std::to_string(ID) + " is missing!");
      }
      if (FCCs.find(ID) == FCCs.end()) {
        ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(h()): FCC for ID="
                                         + std::to_string(ID) + " is missing!");
      }
    }

    // linearize the measurement function with the beliefs:
    std::pair<std::map<size_t, Eigen::MatrixXd>, Eigen::VectorXd> H_r = h(dict_bel, IDs, z);

    // remove fixed measurement Jacobians, beliefs, and FCCs again:
    for (auto iter_bel_I = dict_bel.cbegin(); iter_bel_I != dict_bel.cend() /* not hoisted */; /* no increment */) {
      RTV_EXPECT_TRUE_THROW(iter_bel_I->second != nullptr,
                            "CollaborativeIKFHandler::apply_inter_agent_observation(h()): nullptr in bel_I!");
      if (iter_bel_I->second->options().is_fixed) {
        H_r.first.erase(iter_bel_I->first);  // remove a column of H
        FCCs.erase(iter_bel_I->first);       // remove a row of the FCCs (note: the column persits, but does no harm)
        iter_bel_I = dict_bel.erase(iter_bel_I);
      } else {
        ++iter_bel_I;
      }
    }

    Eigen::MatrixXd H = stack_H(H_r.first);

    Eigen::MatrixXd Sigma_apri = stack_Sigma_locally(dict_bel, t, FCCs);

    // stack individual's covariances:
    Sigma_apri = utils::stabilize_covariance(Sigma_apri);
    bool is_psd = utils::is_positive_semidefinite(Sigma_apri);
    RTV_EXPECT_TRUE_MSG(is_psd, "Joint apri covariance is not PSD at t=" + t.str());
    if (!is_psd) {
      Sigma_apri = utils::nearest_covariance(Sigma_apri, 1e-6);
    }

    KalmanFilter::CorrectionResult_t res;
    ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(h()): correction step");
    res = KalmanFilter::correction_step(H, R, H_r.second, Sigma_apri, cfg);
    if (!res.rejected) {
      bool is_psd = utils::is_positive_semidefinite(res.Sigma_apos);
      RTV_EXPECT_TRUE_MSG(is_psd, "Joint apos covariance is not PSD at t=" + t.str());
      if (!is_psd) {
        res.Sigma_apos = utils::nearest_covariance(res.Sigma_apos, 1e-6);
      }

      // IMPORTANT: MAINTAIN ORDER STRICKTLY
      // 1) LOCAL: add correction terms on all a aprior factorized cross-covariances!
      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::apply_inter_agent_observation(h()): apply_corrections_at_t...");
      apply_corrections_at_t(res.Sigma_apos, dict_bel, t, true);

      // 2) LOCAL: afterwards, overwrite/set factorized a posterioiry cross-covariance (apply no corrections afterwards
      // on them)

      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::apply_inter_agent_observation(h()): split_Sigma_locally...");

      split_Sigma_locally(res.Sigma_apos, dict_bel, FCCs);

      for (auto const &ID_I : local_IDs) {
        if (dict_bel.find(ID_I) != dict_bel.end() && !dict_bel[ID_I]->options().is_fixed) {
          for (auto const &e : FCCs.at(ID_I)) {
            size_t const ID_J = e.first;
            if (dict_bel.find(ID_J) != dict_bel.end() && !dict_bel[ID_J]->options().is_fixed) {
              get(ID_I)->set_CrossCovFact_at_t(t, ID_J, e.second);
            }
          }
        }
      }

      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::apply_inter_agent_observation(h()): correct_beliefs_implace...");

      // 3) correct beliefs implace! will change local_beliefs and remote_beliefs as well!
      correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);

      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::apply_inter_agent_observation(h()): send info to other agent...");

      // 4) send info to other agent:
      remote_FFCs.clear();
      for (auto const &ID_I : remote_IDs) {
        if (dict_bel.find(ID_I) != dict_bel.end() && !dict_bel[ID_I]->options().is_fixed) {
          remote_FFCs[ID_I] = FCCs[ID_I];
        }
      }

      // schedules redo updates automatically
      if (!m_pAgentHandler->set_beliefs_and_FCC_at_t(t, remote_beliefs, remote_FFCs, true, true)) {
        ikf::Logger::ikf_logger()->error(
          "CollaborativeIKFHandler::apply_inter_agent_observation: failed set belief on remote agent...");
        return ApplyObsResult_t(eMeasStatus::DISCARED);
        ;
      } else {
        ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(h()): DONE!");
        return ApplyObsResult_t(eMeasStatus::PROCESSED, H_r.second);
      }
    }

    return ApplyObsResult_t(eMeasStatus::REJECTED, H_r.second);
  } else {
    ikf::Logger::ikf_logger()->error("CollaborativeIKFHandler::apply_inter_agent_observation((h()): mutex FAILED");
    return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
  }
}

}  // namespace ikf
