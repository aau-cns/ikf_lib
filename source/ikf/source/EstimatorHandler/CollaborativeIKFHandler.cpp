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
  : IsolatedKalmanFilterHandler(horizon_sec), m_pAgentHandler(pAgentHdler) {
  Logger::ikf_logger()->info("CollaborativeIKFHandler will inter-agent updates through the agent handler!");
}

size_t ikf::CollaborativeIKFHandler::get_propagation_sensor_ID(const size_t ID) {
  if (ID == 0 || exists(ID)) {
    return m_PropSensor_ID;
  } else {
    return m_pAgentHandler->get_propagation_sensor_ID(ID);
  }
}

bool CollaborativeIKFHandler::get_belief_at_t(const size_t ID, const Timestamp &t, pBelief_t &bel,
                                              const eGetBeliefStrategy type) {
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

std::map<size_t, pBelief_t> CollaborativeIKFHandler::get_dict_bel(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                                  const Timestamp &t) {
  std::map<size_t, pBelief_t> dict_bel;
  for (auto const &e : dict_H) {
    size_t id = e.first;
    pBelief_t bel_apri;
    if (!exists(id)) {
      if (!m_pAgentHandler->get_belief_at_t(id, t, bel_apri, eGetBeliefStrategy::EXACT)) {
        if (!m_pAgentHandler->get_belief_at_t(id, t, bel_apri, eGetBeliefStrategy::LINEAR_INTERPOL_MEAS)) {
          RTV_EXPECT_TRUE_THROW(m_pAgentHandler->get_belief_at_t(id, t, bel_apri, eGetBeliefStrategy::PREDICT_BELIEF),
                                "Could not obtain belief");
        }
      }

    } else {
      if (!get(id)->get_belief_at_t(t, bel_apri, eGetBeliefStrategy::EXACT)) {
        if (!get(id)->get_belief_at_t(t, bel_apri, eGetBeliefStrategy::LINEAR_INTERPOL_MEAS)) {
          RTV_EXPECT_TRUE_THROW(get(id)->get_belief_at_t(t, bel_apri, eGetBeliefStrategy::PREDICT_BELIEF),
                                "Could not obtain belief");
        }
      }
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
                                                     const std::map<size_t, pBelief_t> &dict_bel, const Timestamp &t) {
  size_t row_start = 0;
  for (auto const &e_i : dict_bel) {
    size_t dim_I = e_i.second->es_dim();

    Eigen::MatrixXd Sigma_II_apos = Sigma_apos.block(row_start, row_start, dim_I, dim_I);
    if (exists(e_i.first)) {
      get(e_i.first)->apply_correction_at_t(t, e_i.second->Sigma(), Sigma_II_apos);
    } else {
      m_pAgentHandler->apply_correction_at_t(e_i.first, t, e_i.second->Sigma(), Sigma_II_apos);
    }
    row_start += dim_I;
  }
}

bool CollaborativeIKFHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t,
                                                const KalmanFilter::CorrectionCfg_t &cfg) {
  std::set<IMultiAgentHandler::IDAgent_t> ID_agents;
  std::vector<IMultiAgentHandler::IDEstimator_t> ID_remote_participants;
  for (auto const &e : dict_H) {
    if (!exists(e.first)) {
      ID_agents.insert(m_pAgentHandler->estimatorID2agentID(e.first));
      ID_remote_participants.push_back(e.first);
    }
  }

  // only local instances involved... we are done here!
  if (ID_remote_participants.size() == 0) {
    return IsolatedKalmanFilterHandler::apply_observation(dict_H, R, r, t, cfg);
  }

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

    // IMPORTANT: MAINTAIN ORDER STRICKTLY
    // 1) add correction terms on all a aprior factorized cross-covariances!
    apply_corrections_at_t(res.Sigma_apos, dict_bel, t);

    // 2) afterwards, overwrite/set factorized a posterioiry cross-covariance (apply no corrections afterwards on them)
    split_right_upper_covariance(res.Sigma_apos, dict_bel, t);

    // 3) correct beliefs implace!
    correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);

    // 4) send corrected beliefs back
    for (auto const &id : ID_remote_participants) {
      m_pAgentHandler->set_belief_at_t(id, t, dict_bel.at(id));
    }

    // 5) redo updates after t
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
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());

  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H, R, r, Sigma_apri, cfg);
  if (!res.rejected) {
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(res.Sigma_apos),
                        "Joint apos covariance is not PSD at t=" + t.str());

    // IMPORTANT: MAINTAIN ORDER STRICKTLY
    // 1) add correction terms on all a aprior factorized cross-covariances!
    apply_corrections_at_t(res.Sigma_apos, dict_bel, t);

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

}  // namespace ikf
