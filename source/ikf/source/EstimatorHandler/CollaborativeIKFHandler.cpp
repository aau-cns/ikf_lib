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

namespace ikf {

CollaborativeIKFHandler::CollaborativeIKFHandler(MultiAgentHdl_ptr pAgentHdler, const double horizon_sec)
  : IsolatedKalmanFilterHandler(horizon_sec),
    m_pAgentHandler(pAgentHdler),
    mRedoStrategy(eRedoUpdateStrategy::DISCARD) {
  Logger::ikf_logger()->info("CollaborativeIKFHandler will inter-agent updates through the agent handler!");
  Logger::ikf_logger()->info("* RedoStrategy=" + to_string(mRedoStrategy));
}

size_t ikf::CollaborativeIKFHandler::get_propagation_sensor_ID(const size_t ID) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
  if (ID == 0 || exists(ID)) {
    return m_PropSensor_ID;
  } else {
    // slow: performs a request
    return m_pAgentHandler->get_propagation_sensor_ID(ID);
  }
}

std::string CollaborativeIKFHandler::get_type_by_ID(const size_t ID) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
  if (ID == 0 || exists(ID)) {
    return IDICOHandler::get_type_by_ID(ID);
  } else {
    // fast: accessing locally held data
    return m_pAgentHandler->get_type_by_ID(ID);
  }
}

bool CollaborativeIKFHandler::get_belief_at_t(const size_t ID, const Timestamp &t, pBelief_t &bel,
                                              const eGetBeliefStrategy type) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
  if (exists(ID)) {
    return get(ID)->get_belief_at_t(t, bel, type);
  } else {
    ikf::Logger::ikf_logger()->warn("CollaborativeIKFHandler::get_belief_at_t() from non-local estimator ["
                                    + std::to_string(ID) + "] using the agent handler");
    bool res = m_pAgentHandler->get_belief_at_t(ID, t, bel, type);
    if (!res) {
      ikf::Logger::ikf_logger()->error("CollaborativeIKFHandler::get_belief_at_t() from non-local estimator ["
                                       + std::to_string(ID) + "] failed");
    }
    return res;
  }
}

bool CollaborativeIKFHandler::get_beliefs_at_t(const std::vector<size_t> &IDs,
                                               const std::vector<eGetBeliefStrategy> &types, const Timestamp &t,
                                               std::map<size_t, pBelief_t> &beliefs) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
  std::vector<size_t> local_IDs, remote_IDs;
  std::vector<eGetBeliefStrategy> local_types, remote_types;
  std::map<size_t, pBelief_t> local_beliefs, remote_beliefs;

  for (size_t idx = 0; idx < IDs.size(); idx++) {
    if (exists(IDs.at(idx))) {
      local_IDs.push_back(IDs.at(idx));
      local_types.push_back(types.at(idx));
    } else {
      remote_IDs.push_back(IDs.at(idx));
      remote_types.push_back(types.at(idx));
    }
  }

  bool res = true;
  if (local_IDs.size()) {
    res &= IDICOHandler::get_beliefs_at_t(local_IDs, local_types, t, local_beliefs);
  }
  if (remote_IDs.size()) {
    if (!m_pAgentHandler->get_beliefs_at_t(remote_IDs, remote_types, t, remote_beliefs)) {
      res = false;
      ikf::Logger::ikf_logger()->error("CollaborativeIKFHandler::get_beliefs_at_t() from non-local estimators failed");
    }
  }

  beliefs.clear();
  beliefs.insert(local_beliefs.begin(), local_beliefs.end());
  beliefs.insert(remote_beliefs.begin(), remote_beliefs.end());

  return res;
}

std::map<size_t, pBelief_t> CollaborativeIKFHandler::get_dict_bel(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                                  const Timestamp &t) {
  std::vector<size_t> IDs;
  IDs.reserve(dict_H.size());
  for (auto const &e : dict_H) {
    IDs.push_back(e.first);
  }
  return get_dict_bel(IDs, t);
}

std::map<size_t, pBelief_t> CollaborativeIKFHandler::get_dict_bel(const std::vector<size_t> &IDs, const Timestamp &t) {
  std::map<size_t, pBelief_t> dict_bel;
  for (auto const &e : IDs) {
    size_t id = e;
    pBelief_t bel_apri;
    if (!exists(id)) {
      RTV_EXPECT_TRUE_THROW(
        m_pAgentHandler->get_belief_at_t(id, t, bel_apri, eGetBeliefStrategy::PREDICT_BELIEF),
        "CIKF_Hdl::get_dict_bel(): Could not obtain belief from [" + std::to_string(id) + "] at t=" + t.str());
    } else {
      RTV_EXPECT_TRUE_THROW(
        get(id)->get_belief_at_t(t, bel_apri, eGetBeliefStrategy::PREDICT_BELIEF),
        "CIKF_Hdl::get_dict_bel(): Could not obtain belief from [" + std::to_string(id) + "] at t=" + t.str());
    }

    dict_bel.insert({id, bel_apri});
  }
  return dict_bel;
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

std::set<IMultiAgentHandler::IDAgent_t> CollaborativeIKFHandler::IDs_to_Agent_IDs(const std::set<size_t> &IDs) {
  std::set<IMultiAgentHandler::IDAgent_t> ID_agents;
  for (auto const &ID : IDs) {
    if (!exists(ID)) {
      ID_agents.insert(m_pAgentHandler->estimatorID2agentID(ID));
    }
  }
  return ID_agents;
}

bool CollaborativeIKFHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t,
                                                const KalmanFilter::CorrectionCfg_t &cfg) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
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

    // 2) afterwards, overwrite/set factorized a posterioiry cross-covariance (apply no corrections afterwards on them)
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
        ikf::Logger::ikf_logger()->warn("CollaborativeIKFHandler::apply_observation(): redo_update(agent={:d}) failed",
                                        id);
      }
    }
  }
  return !res.rejected;
}

bool CollaborativeIKFHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                const Eigen::VectorXd &z, const Eigen::MatrixXd &R, const Timestamp &t,
                                                const KalmanFilter::CorrectionCfg_t &cfg) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
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

    // 2) afterwards, overwrite/set factorized a posterioiry cross-covariance (apply no corrections afterwards on them)
    split_right_upper_covariance(res.Sigma_apos, dict_bel, t);

    // 3) correct beliefs implace!
    correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);

    // 4) send corrected beliefs back
    for (auto const &id : ID_remote_participants) {
      m_pAgentHandler->set_belief_at_t(id, t, dict_bel.at(id));
    }
  }
  return !res.rejected;
}

ApplyObsResult_t CollaborativeIKFHandler::apply_observation(const Eigen::MatrixXd &R, const Eigen::VectorXd &z,
                                                            const Timestamp &t, const IIsolatedKalmanFilter::h_joint &h,
                                                            const std::vector<size_t> &IDs,
                                                            const KalmanFilter::CorrectionCfg_t &cfg) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
  std::set<IMultiAgentHandler::IDAgent_t> ID_agents;
  std::vector<IMultiAgentHandler::IDEstimator_t> remote_IDs;
  std::vector<IMultiAgentHandler::IDEstimator_t> local_IDs;

  for (auto const &e : IDs) {
    if (!exists(e)) {
      ID_agents.insert(m_pAgentHandler->estimatorID2agentID(e));
      remote_IDs.push_back(e);
    } else {
      local_IDs.push_back(e);
    }
  }

  ikf::Logger::ikf_logger()->info("CollaborativeIKFHandler::apply_observation(): num remote IDs="
                                  + std::to_string(remote_IDs.size())
                                  + ", num ID_agents=" + std::to_string(ID_agents.size()));

  // only local instances involved... we are done here!
  if (remote_IDs.size() == 0) {
    return IsolatedKalmanFilterHandler::apply_observation(R, z, t, h, IDs, cfg);
  }
  if (ID_agents.size() == 1) {
    // we can perform an optimized update using jumbo messages!

    ikf::Logger::ikf_logger()->debug(
      "CollaborativeIKFHandler::apply_corrections_at_t(): apply_inter_agent_observation...");

    return apply_inter_agent_observation(R, z, t, h, IDs, cfg, remote_IDs, local_IDs);
  }
  return ApplyObsResult_t(eMeasStatus::DISCARED);
}

ProcessMeasResult_vec_t CollaborativeIKFHandler::redo_updates_after_t(const Timestamp &t) {
  // trigger remote agent asynchonrously to redo updates as well...
  // -> hopefully, we are done before remote ones need our beliefs again...
  if (mRedoStrategy == eRedoUpdateStrategy::POSTCORRELATED) {
    auto ID_agents = IDs_to_Agent_IDs(get_remote_correlated_IDs_after_t(t));
    for (auto const &id : IDs_to_Agent_IDs(get_remote_correlated_IDs_after_t(t))) {
      if (!m_pAgentHandler->redo_updates_after_t(id, t)) {
        ikf::Logger::ikf_logger()->warn(
          "CollaborativeIKFHandler::redo_updates_after_t(): redo_update(agent={:d}) failed", id);
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
    // has no effect. Reason: we are not triggering the other agent to redo it's updates after our delayed measurments,
    // we just ignore our delayed measurement in that case.
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
    for (auto const &id : IDs_to_Agent_IDs(get_remote_correlated_IDs_after_t(t))) {
      if (!m_pAgentHandler->redo_updates_after_t(id, t)) {
        ikf::Logger::ikf_logger()->warn(
          "CollaborativeIKFHandler::redo_updates_from_t(): redo_update(agent={:d}) failed", id);
      }
    }
  }
  if (mRedoStrategy == eRedoUpdateStrategy::EXACT) {
    ikf::Logger::ikf_logger()->error("CollaborativeIKFHandler::redo_updates_after_t(): EXACT NOT IMPLEMENTED!");
  }
  return IDICOHandler::redo_updates_from_t(t);
}

bool CollaborativeIKFHandler::apply_inter_agent_observation(
  const std::map<size_t, Eigen::MatrixXd> &dict_H, const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
  const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &remote_IDs,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &local_IDs) {
  std::vector<IMultiAgentHandler::IDEstimator_t> ID_participants = remote_IDs;
  ID_participants.insert(ID_participants.end(), local_IDs.begin(), local_IDs.end());

  Eigen::MatrixXd H = stack_H(dict_H);
  std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> remote_beliefs, local_beliefs;
  std::map<size_t, std::map<size_t, Eigen::MatrixXd>> remote_FFCs, local_FFCs;
  if (!get_local_beliefs_and_FCC_at_t(local_IDs, ID_participants, t, local_beliefs, local_FFCs)) {
    ikf::Logger::ikf_logger()->error(
      "CollaborativeIKFHandler::apply_inter_agent_observation: failed to obtain local data...");
    return false;
  }
  if (!m_pAgentHandler->get_beliefs_and_FCC_at_t(remote_IDs, ID_participants, t, remote_beliefs, remote_FFCs)) {
    ikf::Logger::ikf_logger()->error(
      "CollaborativeIKFHandler::apply_inter_agent_observation: failed to obtain remote data...");
    return false;
  }

  ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation() stack beliefs...");

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
  ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(): correction step");
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
      "CollaborativeIKFHandler::apply_inter_agent_observation(): apply_corrections_at_t...");
    apply_corrections_at_t(res.Sigma_apos, dict_bel, t, true);

    // 2) LOCAL: afterwards, overwrite/set factorized a posterioiry cross-covariance (apply no corrections afterwards on
    // them)

    ikf::Logger::ikf_logger()->debug(
      "CollaborativeIKFHandler::apply_inter_agent_observation(): split_Sigma_locally...");

    split_Sigma_locally(res.Sigma_apos, dict_bel, FCCs);

    for (auto const &ID_I : local_IDs) {
      for (auto const &e : FCCs.at(ID_I)) {
        size_t const ID_J = e.first;
        get(ID_I)->set_CrossCovFact_at_t(t, ID_J, e.second);
      }
    }

    ikf::Logger::ikf_logger()->debug(
      "CollaborativeIKFHandler::apply_inter_agent_observation(): correct_beliefs_implace...");

    // 3) correct beliefs implace! will change local_beliefs and remote_beliefs as well!
    correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);

    ikf::Logger::ikf_logger()->debug(
      "CollaborativeIKFHandler::apply_inter_agent_observation(): send info to other agent...");

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
      ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(): DONE!");
      return true;
    }
  }

  return true;
}

ApplyObsResult_t CollaborativeIKFHandler::apply_inter_agent_observation(
  const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t, const IIsolatedKalmanFilter::h_joint &h,
  const std::vector<size_t> &IDs, const KalmanFilter::CorrectionCfg_t &cfg,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &remote_IDs,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &local_IDs) {
  std::vector<IMultiAgentHandler::IDEstimator_t> ID_participants = remote_IDs;
  ID_participants.insert(ID_participants.end(), local_IDs.begin(), local_IDs.end());

  std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> remote_beliefs, local_beliefs;
  std::map<size_t, std::map<size_t, Eigen::MatrixXd>> remote_FFCs, local_FFCs;
  if (!get_local_beliefs_and_FCC_at_t(local_IDs, ID_participants, t, local_beliefs, local_FFCs)) {
    ikf::Logger::ikf_logger()->error(
      "CollaborativeIKFHandler::apply_inter_agent_observation: failed to obtain local data...");
    return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
  }
  if (!m_pAgentHandler->get_beliefs_and_FCC_at_t(remote_IDs, ID_participants, t, remote_beliefs, remote_FFCs)) {
    ikf::Logger::ikf_logger()->error(
      "CollaborativeIKFHandler::apply_inter_agent_observation: failed to obtain remote data...");
    return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
  }

  ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation() stack beliefs...");

  // STACK BELIEFS AND Factorized-Cross-Covariances (FFCs)
  std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> dict_bel;
  dict_bel.insert(local_beliefs.begin(), local_beliefs.end());
  dict_bel.insert(remote_beliefs.begin(), remote_beliefs.end());
  std::map<size_t, std::map<size_t, Eigen::MatrixXd>> FCCs;
  FCCs.insert(local_FFCs.begin(), local_FFCs.end());
  FCCs.insert(remote_FFCs.begin(), remote_FFCs.end());

  for (auto ID : IDs) {
    if (dict_bel.find(ID) == dict_bel.end()) {
      ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(): belief for ID="
                                       + std::to_string(ID) + " is missing!");
    }
    if (FCCs.find(ID) == FCCs.end()) {
      ikf::Logger::ikf_logger()->debug(
        "CollaborativeIKFHandler::apply_inter_agent_observation(): FCC for ID=" + std::to_string(ID) + " is missing!");
    }
  }

  // linearize the measurement function with the beliefs:
  std::pair<std::map<size_t, Eigen::MatrixXd>, Eigen::VectorXd> H_r = h(dict_bel, IDs, z);
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
  ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(): correction step");
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
      "CollaborativeIKFHandler::apply_inter_agent_observation(): apply_corrections_at_t...");
    apply_corrections_at_t(res.Sigma_apos, dict_bel, t, true);

    // 2) LOCAL: afterwards, overwrite/set factorized a posterioiry cross-covariance (apply no corrections afterwards on
    // them)

    ikf::Logger::ikf_logger()->debug(
      "CollaborativeIKFHandler::apply_inter_agent_observation(): split_Sigma_locally...");

    split_Sigma_locally(res.Sigma_apos, dict_bel, FCCs);

    for (auto const &ID_I : local_IDs) {
      for (auto const &e : FCCs.at(ID_I)) {
        size_t const ID_J = e.first;
        get(ID_I)->set_CrossCovFact_at_t(t, ID_J, e.second);
      }
    }

    ikf::Logger::ikf_logger()->debug(
      "CollaborativeIKFHandler::apply_inter_agent_observation(): correct_beliefs_implace...");

    // 3) correct beliefs implace! will change local_beliefs and remote_beliefs as well!
    correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);

    ikf::Logger::ikf_logger()->debug(
      "CollaborativeIKFHandler::apply_inter_agent_observation(): send info to other agent...");

    // 4) send info to other agent:
    remote_FFCs.clear();
    for (auto const &ID_I : remote_IDs) {
      remote_FFCs[ID_I] = FCCs[ID_I];
    }

    // schedules redo updates automatically
    if (!m_pAgentHandler->set_beliefs_and_FCC_at_t(t, remote_beliefs, remote_FFCs, true, true)) {
      ikf::Logger::ikf_logger()->error(
        "CollaborativeIKFHandler::apply_inter_agent_observation: failed set belief on remote agent...");
      return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
      ;
    } else {
      ikf::Logger::ikf_logger()->debug("CollaborativeIKFHandler::apply_inter_agent_observation(): DONE!");
      return ApplyObsResult_t(eMeasStatus::PROCESSED, H_r.second);
    }
  }

  return ApplyObsResult_t(eMeasStatus::REJECTED, H_r.second);
}

bool CollaborativeIKFHandler::get_local_beliefs_and_FCC_at_t(
  const std::vector<IMultiAgentHandler::IDEstimator_t> &IDs,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &ID_participants, const Timestamp &t,
  std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> &beliefs,
  std::map<size_t, std::map<size_t, Eigen::MatrixXd>> &dict_FFC) {
  beliefs.clear();
  dict_FFC.clear();
  for (size_t idx = 0; idx < IDs.size(); idx++) {
    auto ID_I = IDs.at(idx);
    if (exists(ID_I)) {
      ikf::pBelief_t pBel;
      ikf::eGetBeliefStrategy type = ikf::eGetBeliefStrategy::PREDICT_BELIEF;
      if (get(ID_I)->get_belief_at_t(t, pBel, type)) {
        beliefs.insert({ID_I, pBel});
      } else {
        ikf::Logger::ikf_logger()->error(
          "CollaborativeIKFHandler::get_local_beliefs_and_FCC_at_t: could not get_belief_at_t for ID=[{:}]", ID_I);
        return false;
      }
    } else {
      ikf::Logger::ikf_logger()->error(
        "CollaborativeIKFHandler::get_local_beliefs_and_FCC_at_t: unknown local ID=[{:}]", ID_I);
      return false;
    }
  }  // for-loop local ids:
  for (size_t const &ID_I : IDs) {
    for (size_t const &ID_P : ID_participants) {
      if (ID_I != ID_P) {
        if (exists(ID_I)) {
          Eigen::MatrixXd FFC_IJ = get(ID_I)->get_CrossCovFact_at_t(t, ID_P);

          if (dict_FFC.find(ID_I) == dict_FFC.end()) {
            dict_FFC[ID_I] = std::map<size_t, Eigen::MatrixXd>();
          }
          dict_FFC[ID_I].insert({ID_P, FFC_IJ});
        } else {
          ikf::Logger::ikf_logger()->error(
            "CollaborativeIKFHandler::get_local_beliefs_and_FCC_at_t: unknown local ID=[{:}]", ID_I);
          return false;
        }
      }
    }
  }
  return true;
}

Eigen::MatrixXd CollaborativeIKFHandler::stack_Sigma_locally(
  const std::map<size_t, pBelief_t> &dict_bel, const Timestamp &t,
  std::map<size_t, std::map<size_t, Eigen::MatrixXd>> &dict_FFC) {
  size_t state_dim = 0;
  for (auto const &e : dict_bel) {
    state_dim += e.second->es_dim();
  }
  Eigen::MatrixXd Sigma = Eigen::MatrixXd::Zero(state_dim, state_dim);

  RTV_EXPECT_TRUE_THROW(dict_FFC.size() == dict_bel.size(),
                        "CollaborativeIKFHandler::stack_Sigma_locally: not enough rows in FFCs");

  size_t row_start = 0;
  for (auto const &e_i : dict_bel) {
    size_t id_row = e_i.first;
    size_t state_dim_row = e_i.second->es_dim();
    size_t col_start = 0;
    RTV_EXPECT_TRUE_THROW(dict_FFC[id_row].size() == dict_bel.size() - 1,
                          "CollaborativeIKFHandler::stack_Sigma_locally: not enough cols in FFCs");
    for (auto const &e_j : dict_bel) {
      size_t id_col = e_j.first;
      size_t state_dim_col = e_j.second->es_dim();
      if (id_row == id_col) {
        Sigma.block(row_start, col_start, state_dim_row, state_dim_col) = e_i.second->Sigma();
      } else {
        // obtain only the upper triangular part (if dict_bel was sorted ascending)
        if (id_row < id_col) {
          Eigen::MatrixXd SigmaFact_IJ = dict_FFC.at(id_row).at(id_col);
          Eigen::MatrixXd SigmaFact_JI = dict_FFC.at(id_col).at(id_row);
          Eigen::MatrixXd Sigma_IJ;
          if (SigmaFact_IJ.size() && SigmaFact_JI.size()) {
            RTV_EXPECT_TRUE_MSG(SigmaFact_IJ.cols() == SigmaFact_JI.cols(), "get_crosscov wrong dims @t=" + t.str());
            Sigma_IJ = SigmaFact_IJ * SigmaFact_JI.transpose();
          }
          if (Sigma_IJ.size()) {
            Sigma.block(row_start, col_start, state_dim_row, state_dim_col) = Sigma_IJ;
            Sigma.block(col_start, row_start, state_dim_col, state_dim_row) = Sigma_IJ.transpose();
          }
        }
      }
      col_start += state_dim_col;
    }
    row_start += state_dim_row;
  }

  return Sigma;
}

void CollaborativeIKFHandler::split_Sigma_locally(Eigen::MatrixXd &Sigma, const std::map<size_t, pBelief_t> &dict_bel,
                                                  std::map<size_t, std::map<size_t, Eigen::MatrixXd>> &dict_FCC) {
  std::vector<size_t> IDs;
  for (auto const &e : dict_bel) {
    IDs.push_back(e.first);
  }

  dict_FCC.clear();

  size_t row_start = 0, col_start_offset = 0;
  for (size_t i = 0; i < IDs.size() - 1; i++) {
    size_t dim_i = dict_bel.at(IDs.at(i))->es_dim();
    col_start_offset += dim_i;
    size_t ID_I = IDs.at(i);
    size_t col_start = col_start_offset;
    for (size_t j = i + 1; j < IDs.size(); j++) {
      size_t dim_j = dict_bel.at(IDs.at(j))->es_dim();
      size_t ID_J = IDs.at(j);
      Eigen::MatrixXd Sigma_IJ = Sigma.block(row_start, col_start, dim_i, dim_j);
      // set_Sigma_IJ_at_t(ID_I, ID_J, Sigma_IJ_apos, t);
      Eigen::MatrixXd FCC_JI, FCC_IJ;
      if (ID_I < ID_J) {
        FCC_IJ = Sigma_IJ;
        FCC_JI = Eigen::MatrixXd::Identity(FCC_IJ.cols(), FCC_IJ.cols());
      } else {
        FCC_JI = Sigma_IJ.transpose();
        FCC_IJ = Eigen::MatrixXd::Identity(FCC_JI.cols(), FCC_JI.cols());
      }

      // insert FCCs in dictionary
      if (dict_FCC.find(ID_I) == dict_FCC.end()) {
        dict_FCC[ID_I] = std::map<size_t, Eigen::MatrixXd>();
      }
      dict_FCC[ID_I].insert({ID_J, FCC_IJ});

      if (dict_FCC.find(ID_J) == dict_FCC.end()) {
        dict_FCC[ID_J] = std::map<size_t, Eigen::MatrixXd>();
      }
      dict_FCC[ID_J].insert({ID_I, FCC_JI});

      col_start += dim_j;
    }
    row_start += dim_i;
  }
}

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

}  // namespace ikf
