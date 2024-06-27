/******************************************************************************
 * FILENAME:     ICSE_IKF_Handler.cpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     27.06.2024
 *
 *  Copyright (C) 2024
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#include <ikf/EstimatorHandler/ICSE_IKF_Handler.hpp>
#include <ikf/Logger/Logger.hpp>
#include <ikf/utils/eigen_utils.hpp>
#include <ikf/utils/lock_guard_timed.hpp>
namespace ikf {

ICSE_IKF_Handler::ICSE_IKF_Handler(MultiAgentHdl_ptr pAgentHdler, const double horizon_sec)
  : IsolatedKalmanFilterHandler(horizon_sec), m_pAgentHandler(pAgentHdler) {
  Logger::ikf_logger()->info("ICSE_IKF_Handler: will inter-agent updates through the agent handler!");
}

size_t ikf::ICSE_IKF_Handler::get_propagation_sensor_ID(const size_t ID) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    if (ID == 0 || exists(ID)) {
      return m_PropSensor_ID;
    } else {
      // slow: performs a request
      return m_pAgentHandler->get_propagation_sensor_ID(ID);
    }
  } else {
    ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::get_propagation_sensor_ID(): mutex FAILED");
    return 0;
  }
}

std::string ICSE_IKF_Handler::get_type_by_ID(const size_t ID) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    if (ID == 0 || exists(ID)) {
      return IDICOHandler::get_type_by_ID(ID);
    } else {
      // fast: accessing locally held data
      return m_pAgentHandler->get_type_by_ID(ID);
    }
  } else {
    ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::get_type_by_ID(): mutex FAILED");
    return std::string("");
  }
}

bool ICSE_IKF_Handler::get_belief_at_t(const size_t ID, const Timestamp &t, pBelief_t &bel,
                                       const eGetBeliefStrategy type) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    if (exists(ID)) {
      return get(ID)->get_belief_at_t(t, bel, type);
    } else {
      ikf::Logger::ikf_logger()->warn("ICSE_IKF_Handler::get_belief_at_t() from non-local estimator ["
                                      + std::to_string(ID) + "] using the agent handler");
      bool res = m_pAgentHandler->get_belief_at_t(ID, t, bel, type);
      if (!res) {
        ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::get_belief_at_t() from non-local estimator ["
                                         + std::to_string(ID) + "] failed");
      }
      return res;
    }
  } else {
    ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::get_belief_at_t(): mutex FAILED");
    return false;
  }
}

bool ICSE_IKF_Handler::get_beliefs_at_t(const std::vector<size_t> &IDs, const std::vector<eGetBeliefStrategy> &types,
                                        const Timestamp &t, std::map<size_t, pBelief_t> &beliefs) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
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
        ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::get_beliefs_at_t() from non-local estimators failed");
      }
    }

    beliefs.clear();
    beliefs.insert(local_beliefs.begin(), local_beliefs.end());
    beliefs.insert(remote_beliefs.begin(), remote_beliefs.end());

    return res;
  } else {
    ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::get_beliefs_at_t(): mutex FAILED");
    return false;
  }
}

ApplyObsResult_t ICSE_IKF_Handler::apply_observation(const Eigen::MatrixXd &R, const Eigen::VectorXd &z,
                                                     const Timestamp &t, const IIsolatedKalmanFilter::h_joint &h,
                                                     const std::vector<size_t> &IDs,
                                                     const KalmanFilter::CorrectionCfg_t &cfg) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
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

    ikf::Logger::ikf_logger()->info(
      "ICSE_IKF_Handler::apply_observation():  num local IDs=" + std::to_string(local_IDs.size())
      + "num remote IDs=" + std::to_string(remote_IDs.size()) + ", num ID_agents=" + std::to_string(ID_agents.size()));

    // only local instances involved... we are done here!
    if (remote_IDs.size() == 0) {
      return IsolatedKalmanFilterHandler::apply_observation(R, z, t, h, IDs, cfg);
    }
    if (ID_agents.size() == 1) {
      // we can perform an optimized update using jumbo messages!

      ikf::Logger::ikf_logger()->debug("ICSE_IKF_Handler::apply_corrections_at_t(): apply_inter_agent_observation...");

      // call method of child:
      return apply_inter_agent_observation(R, z, t, h, IDs, cfg, remote_IDs, local_IDs);
    }
    return ApplyObsResult_t(eMeasStatus::DISCARED);
  } else {
    ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::apply_observation(): mutex FAILED");
    return ApplyObsResult_t(eMeasStatus::OUTOFORDER);
  }
}

std::set<IMultiAgentHandler::IDAgent_t> ICSE_IKF_Handler::IDs_to_Agent_IDs(const std::set<size_t> &IDs) {
  std::set<IMultiAgentHandler::IDAgent_t> ID_agents;
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    for (auto const &ID : IDs) {
      if (!exists(ID)) {
        ID_agents.insert(m_pAgentHandler->estimatorID2agentID(ID));
      }
    }
  } else {
    ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::IDs_to_Agent_IDs(): mutex FAILED");
  }
  return ID_agents;
}

std::map<size_t, pBelief_t> ICSE_IKF_Handler::get_dict_bel(const std::vector<size_t> &ids, const Timestamp &t) {
  std::map<size_t, pBelief_t> dict_bel;
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    for (auto const &e : ids) {
      size_t id = e;
      pBelief_t bel_apri;
      if (!exists(id)) {
        // RTV_EXPECT_TRUE_THROW(
        //   m_pAgentHandler->get_belief_at_t(id, t, bel_apri, eGetBeliefStrategy::PREDICT_BELIEF),
        //   "CIKF_Hdl::get_dict_bel(): Could not obtain belief from [" + std::to_string(id) + "] at t=" + t.str());
        if (!m_pAgentHandler->get_belief_at_t(id, t, bel_apri, eGetBeliefStrategy::PREDICT_BELIEF)) {
          //  OUT OF ORDER MEASUREMENT
          return std::map<size_t, pBelief_t>();
        }
      } else {
        // RTV_EXPECT_TRUE_THROW(
        //   get(id)->get_belief_at_t(t, bel_apri, eGetBeliefStrategy::PREDICT_BELIEF),
        //   "CIKF_Hdl::get_dict_bel(): Could not obtain belief from [" + std::to_string(id) + "] at t=" + t.str());
        if (!get(id)->get_belief_at_t(t, bel_apri, eGetBeliefStrategy::PREDICT_BELIEF)) {
          //  OUT OF ORDER MEASUREMENT
          return std::map<size_t, pBelief_t>();
        }
      }

      dict_bel.insert({id, bel_apri});
    }
  } else {
    ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::get_dict_bel(): mutex FAILED");
  }
  return dict_bel;
}

bool ICSE_IKF_Handler::get_local_beliefs_and_FCC_at_t(
  const std::vector<IMultiAgentHandler::IDEstimator_t> &IDs,
  const std::vector<IMultiAgentHandler::IDEstimator_t> &ID_participants, const Timestamp &t,
  std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t> &beliefs,
  std::map<size_t, std::map<size_t, Eigen::MatrixXd> > &dict_FFC) {
  //
  beliefs.clear();
  dict_FFC.clear();
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    for (size_t idx = 0; idx < IDs.size(); idx++) {
      auto ID_I = IDs.at(idx);
      if (exists(ID_I)) {
        ikf::pBelief_t pBel;
        ikf::eGetBeliefStrategy type = ikf::eGetBeliefStrategy::PREDICT_BELIEF;
        if (get(ID_I)->get_belief_at_t(t, pBel, type)) {
          beliefs.insert({ID_I, pBel});
        } else {
          ikf::Logger::ikf_logger()->error(
            "ICSE_IKF_Handler::get_local_beliefs_and_FCC_at_t: could not get_belief_at_t for ID=[{:}]", ID_I);
          return false;
        }
      } else {
        ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::get_local_beliefs_and_FCC_at_t: unknown local ID=[{:}]",
                                         ID_I);
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
            ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::get_local_beliefs_and_FCC_at_t: unknown local ID=[{:}]",
                                             ID_I);
            return false;
          }
        }
      }
    }
    return true;
  } else {
    ikf::Logger::ikf_logger()->error("ICSE_IKF_Handler::get_local_beliefs_and_FCC_at_t(): mutex FAILED");
    return false;
  }
}

Eigen::MatrixXd ICSE_IKF_Handler::stack_Sigma_locally(const std::map<size_t, pBelief_t> &dict_bel, const Timestamp &t,
                                                      std::map<size_t, std::map<size_t, Eigen::MatrixXd> > &dict_FFC) {
  size_t state_dim = 0;
  for (auto const &e : dict_bel) {
    state_dim += e.second->es_dim();
  }
  Eigen::MatrixXd Sigma = Eigen::MatrixXd::Zero(state_dim, state_dim);

  RTV_EXPECT_TRUE_THROW(dict_FFC.size() == dict_bel.size(),
                        "ICSE_IKF_Handler::stack_Sigma_locally: not enough rows in FFCs");

  size_t row_start = 0;
  for (auto const &e_i : dict_bel) {
    size_t id_row = e_i.first;
    size_t state_dim_row = e_i.second->es_dim();
    size_t col_start = 0;
    RTV_EXPECT_TRUE_THROW(dict_FFC[id_row].size() >= dict_bel.size() - 1,
                          "ICSE_IKF_Handler::stack_Sigma_locally: not enough cols in FFCs");
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

void ICSE_IKF_Handler::split_Sigma_locally(Eigen::MatrixXd &Sigma, const std::map<size_t, pBelief_t> &dict_bel,
                                           std::map<size_t, std::map<size_t, Eigen::MatrixXd> > &dict_FCC) {
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

}  // namespace ikf
