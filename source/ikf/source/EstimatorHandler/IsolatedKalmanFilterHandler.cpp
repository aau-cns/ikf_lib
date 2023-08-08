/******************************************************************************
* FILENAME:     IsolatedKalmanFilterHandler.cpp
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
#include <ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp>
#include <ikf/Logger/Logger.hpp>
#include <ikf/utils/eigen_utils.hpp>

namespace ikf {

IsolatedKalmanFilterHandler::IsolatedKalmanFilterHandler(const bool handle_delayed, const double horizon_sec)
  : m_handle_delayed_meas(true), HistMeas(horizon_sec), m_horzion_sec(horizon_sec) {
  Logger::ikf_logger()->info(
    "IsolatedKalmanFilterHandler will handle delayed measurements, therefore call it's process_measurement method!");

  Logger::ikf_logger()->info("IsolatedKalmanFilterHandler: m_horizon_sec=" + std::to_string(m_horzion_sec));
}

bool IsolatedKalmanFilterHandler::add(pIKF_t p_IKF) {
  if (!exists(p_IKF->ID())) {
    // either one is handling delayed measurements!
    p_IKF->handle_delayed_meas(false);
    p_IKF->set_horizon(m_horzion_sec);
    id_dict.emplace(p_IKF->ID(), p_IKF);
    return true;
  }
  return false;
}

bool IsolatedKalmanFilterHandler::remove(const size_t ID) {
  auto elem = get(ID);
  if (elem) {
    id_dict.erase(elem->ID());
    return true;
  }
  return false;
}

bool IsolatedKalmanFilterHandler::exists(const size_t ID) {
  return (id_dict.find(ID) != id_dict.end());
}

std::vector<size_t> IsolatedKalmanFilterHandler::get_instance_ids() {
  std::vector<size_t> IDs;
  IDs.reserve(id_dict.size());
  for (auto const& elem : id_dict) {
    IDs.push_back(elem.first);
  }
  return IDs;
}

double ikf::IsolatedKalmanFilterHandler::horizon_sec() const { return m_horzion_sec; }

void IsolatedKalmanFilterHandler::set_horizon(const double t_hor) {
  m_horzion_sec = t_hor;
  for (auto const &elem : id_dict) {
    elem.second->set_horizon(m_horzion_sec);
  }
  HistMeas.set_horizon(m_horzion_sec);
}

bool ikf::IsolatedKalmanFilterHandler::insert_measurement(const MeasData &m, const Timestamp &t) {
  HistMeas.insert(m, t);
  return true;
}

void IsolatedKalmanFilterHandler::sort_measurements_from_t(const Timestamp &t) {
  Timestamp t_latest;
  if(HistMeas.get_latest_t(t_latest)) {
    TMultiHistoryBuffer<MeasData> meas = HistMeas.get_between_t1_t2(t, t_latest);

    if(!meas.empty()) {
      HistMeas.remove_after_t(t);
      HistMeas.remove_at_t(t);

      // first insert all PROPAGATION sorted
      meas.foreach ([this](MeasData const &elem) {
        if (elem.obs_type == eObservationType::PROPAGATION) {
          HistMeas.insert(elem, elem.t_m);
        }
      });

      // second insert all PRIVATE sorted
      meas.foreach ([this](MeasData const &elem) {
        if (elem.obs_type == eObservationType::PRIVATE_OBSERVATION) {
          HistMeas.insert(elem, elem.t_m);
        }
      });

      // third insert all JOINT sorted
      meas.foreach ([this](MeasData const &elem) {
        if (elem.obs_type == eObservationType::JOINT_OBSERVATION) {
          HistMeas.insert(elem, elem.t_m);
        }
      });
    }

  }
}

ProcessMeasResult_t IsolatedKalmanFilterHandler::process_measurement(const MeasData &m) {
  ProcessMeasResult_t res = reprocess_measurement(m);
  bool order_violated = is_order_violated(m);
  if (order_violated) {
    HistMeas.insert(m, m.t_m);
    sort_measurements_from_t(m.t_m);
    redo_updates_from_t(m.t_m);
  } else {
    if (!res.rejected && HistMeas.exist_after_t(m.t_m)) {
      redo_updates_after_t(m.t_m);
    }
    HistMeas.insert(m, m.t_m);
  }
  HistMeas.check_horizon();
  return res;
}

ProcessMeasResult_t IsolatedKalmanFilterHandler::reprocess_measurement(const MeasData &m) {
  ProcessMeasResult_t res;
  if (exists(m.id_sensor)) {
    res = id_dict[m.id_sensor]->reprocess_measurement(m);
  }
  return res;
}

bool IsolatedKalmanFilterHandler::redo_updates_from_t(const Timestamp &t) {
  remove_beliefs_from_t(t);
  Timestamp t_last;
  if (HistMeas.get_latest_t(t_last)) {
    Logger::ikf_logger()->info("IsolatedKalmanFilterHandler::redo_updates_from_t() t=" + t.str() + ", t_last=" + t_last.str());

    if (t == t_last) {
      auto vec = HistMeas.get_all_at_t(t);
      for (MeasData &m : vec) {
        this->reprocess_measurement(m);
      }
    }
    else {
      HistMeas.foreach_between_t1_t2(t, t_last, [this](MeasData const&m){ this->reprocess_measurement(m); });
    }
    return true;
  }
  return false;
}

bool IsolatedKalmanFilterHandler::redo_updates_after_t(const Timestamp &t) {
  remove_beliefs_after_t(t);
  Timestamp t_after, t_last;
  if (HistMeas.get_after_t(t, t_after) &&  HistMeas.get_latest_t(t_last)) {
    Logger::ikf_logger()->info("IsolatedKalmanFilterHandler::redo_updates_after_t() t_after=" + t_after.str() + ", t_last=" + t_last.str());
    if (t_after == t_last) {
      auto vec = HistMeas.get_all_at_t(t_after);
      for (MeasData &m : vec) {
        this->reprocess_measurement(m);
      }
    }
    else {
      HistMeas.foreach_between_t1_t2(t_after, t_last, [this](MeasData const&m){ this->reprocess_measurement(m); });
    }
    return true;
  }
  return false;
}

void IsolatedKalmanFilterHandler::remove_beliefs_after_t(const Timestamp &t) {
  for (auto& elem : id_dict) {
    elem.second->remove_after_t(t);
  }
}

void IsolatedKalmanFilterHandler::remove_beliefs_from_t(const Timestamp &t) {
  for (auto& elem : id_dict) {
    elem.second->remove_from_t(t);
  }
}

std::shared_ptr<IIsolatedKalmanFilter> IsolatedKalmanFilterHandler::get(const size_t ID) {
  auto it = id_dict.find(ID);
  if (it != id_dict.end()) {
    return it->second;
  }
  return std::shared_ptr<IIsolatedKalmanFilter>(nullptr);
}


void IsolatedKalmanFilterHandler::reset() {
  for (auto& elem : id_dict) {
    elem.second->reset();
  }
}

bool IsolatedKalmanFilterHandler::is_order_violated(const MeasData &m) {
  if (m.obs_type != eObservationType::JOINT_OBSERVATION) {
    auto meas_arr = HistMeas.get_all_at_t(m.t_m);

    if (m.obs_type == eObservationType::PROPAGATION) {
      for (MeasData & m_ : meas_arr) {
        if (m_.obs_type == eObservationType::PRIVATE_OBSERVATION ||
            m_.obs_type == eObservationType::JOINT_OBSERVATION ){
          return true;
        }
      }
    } else if  (m.obs_type == eObservationType::PRIVATE_OBSERVATION) {
      for (MeasData & m_ : meas_arr) {
        if (m_.obs_type == eObservationType::JOINT_OBSERVATION) {
          return true;
        }
      }
    }
  }
  return false;
}

// Algorithm 6 in [1]
Eigen::MatrixXd IsolatedKalmanFilterHandler::stack_apri_covariance(pBelief_t &bel_I_apri, pBelief_t &bel_J_apri,
                                                                   const size_t ID_I, const size_t ID_J,
                                                                   const Timestamp &t) {
  Eigen::MatrixXd Sigma_IJ = get_Sigma_IJ_at_t(ID_I, ID_J, t);

  if (Sigma_IJ.size()) {
    return utils::stack_Sigma(bel_I_apri->Sigma(), bel_J_apri->Sigma(), Sigma_IJ);
  } else {
    // Not correlated!
    Sigma_IJ = Sigma_IJ.Zero(bel_I_apri->es_dim(), bel_J_apri->es_dim());
    return utils::stack_Sigma(bel_I_apri->Sigma(), bel_J_apri->Sigma(), Sigma_IJ);
  }
}

Eigen::MatrixXd ikf::IsolatedKalmanFilterHandler::stack_apri_covariance(pBelief_t &bel_I_apri, pBelief_t &bel_J_apri,
                                                                        pBelief_t &bel_K_apri, const size_t ID_I,
                                                                        const size_t ID_J, const size_t ID_K,
                                                                        const Timestamp &t) {
  // stack individual's covariances:
  Eigen::MatrixXd Sigma_apri_22 = stack_apri_covariance(bel_I_apri, bel_J_apri, ID_I, ID_J, t);

  Eigen::MatrixXd Sigma_IK = get_Sigma_IJ_at_t(ID_I, ID_K, t);
  if (!Sigma_IK.size()) {
    Sigma_IK = Sigma_IK.Zero(bel_I_apri->es_dim(), bel_K_apri->es_dim());
  }
  Eigen::MatrixXd Sigma_JK = get_Sigma_IJ_at_t(ID_J, ID_K, t);
  if (!Sigma_JK.size()) {
    Sigma_JK = Sigma_JK.Zero(bel_J_apri->es_dim(), bel_K_apri->es_dim());
  }

  Eigen::MatrixXd Sigma_2K_apri = utils::vertcat(Sigma_IK, Sigma_JK);
  return utils::stack_Sigma(Sigma_apri_22, bel_K_apri->Sigma(), Sigma_2K_apri);
}

Eigen::MatrixXd IsolatedKalmanFilterHandler::stack_apri_covariance(pBelief_t &bel_I_apri, pBelief_t &bel_J_apri,
                                                                   pBelief_t &bel_K_apri, pBelief_t &bel_L_apri,
                                                                   const size_t ID_I, const size_t ID_J,
                                                                   const size_t ID_K, const size_t ID_L,
                                                                   const Timestamp &t) {
  // stack individual's covariances:
  Eigen::MatrixXd Sigma_apri_33 = stack_apri_covariance(bel_I_apri, bel_J_apri, bel_K_apri, ID_I, ID_J, ID_K, t);

  Eigen::MatrixXd Sigma_IL = get_Sigma_IJ_at_t(ID_I, ID_L, t);
  if (!Sigma_IL.size()) {
    Sigma_IL = Sigma_IL.Zero(bel_I_apri->es_dim(), bel_K_apri->es_dim());
  }
  Eigen::MatrixXd Sigma_JL = get_Sigma_IJ_at_t(ID_J, ID_L, t);
  if (!Sigma_JL.size()) {
    Sigma_JL = Sigma_JL.Zero(bel_J_apri->es_dim(), bel_K_apri->es_dim());
  }

  Eigen::MatrixXd Sigma_KL = get_Sigma_IJ_at_t(ID_K, ID_L, t);
  if (!Sigma_KL.size()) {
    Sigma_KL = Sigma_KL.Zero(bel_J_apri->es_dim(), bel_K_apri->es_dim());
  }

  Eigen::MatrixXd Sigma_3L_apri = utils::vertcat(Sigma_IL, Sigma_JL, Sigma_KL);
  return utils::stack_Sigma(Sigma_apri_33, bel_L_apri->Sigma(), Sigma_3L_apri);
}

//  Eq (1) and Algorithm 6 in [1]
void IsolatedKalmanFilterHandler::set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J,
                                                    const Eigen::MatrixXd &Sigma_IJ, const Timestamp &t) {
  if (ID_I < ID_J) {
    get(ID_I)->set_CrossCovFact_at_t(t, ID_J, Sigma_IJ);
    get(ID_J)->set_CrossCovFact_at_t(t, ID_I, Eigen::MatrixXd::Identity(Sigma_IJ.cols(), Sigma_IJ.cols()));
  } else {
    Eigen::MatrixXd Sigma_JI = Sigma_IJ.transpose();
    get(ID_J)->set_CrossCovFact_at_t(t, ID_I, Sigma_JI);
    get(ID_I)->set_CrossCovFact_at_t(t, ID_J, Eigen::MatrixXd::Identity(Sigma_JI.cols(), Sigma_JI.cols()));
  }
}

//  Eq (1) and Algorithm 6 in [1]
Eigen::MatrixXd IsolatedKalmanFilterHandler::get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J,
                                                               const Timestamp &t) {
  Eigen::MatrixXd SigmaFact_IJ = get(ID_I)->get_CrossCovFact_at_t(t, ID_J);
  Eigen::MatrixXd SigmaFact_JI = get(ID_J)->get_CrossCovFact_at_t(t, ID_I);
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
bool IsolatedKalmanFilterHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                    const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
                                                    const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg) {
  std::map<size_t, size_t> dict_dim;
  std::vector<size_t> IDs;
  size_t state_dim = 0;
  size_t H_rows = 0;
  for (auto const &e : dict_H) {
    IDs.push_back(e.first);
    dict_dim.insert({e.first, e.second.cols()});
    state_dim += e.second.cols();
    H_rows = e.second.rows();
  }

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(H_rows, state_dim);
  {
    size_t col_start = 0;
    for (auto const &id : IDs) {
      H.block(0, col_start, H_rows, dict_dim[id]) = dict_H.at(id);
      col_start += dict_dim[id];
    }
  }

  std::map<size_t, pBelief_t> dict_bel;
  for (auto const &id : IDs) {
    pBelief_t bel_apri;
    if (!get(id)->get_belief_at_t(t, bel_apri)) {
      RTV_EXPECT_TRUE_MSG(false, "No belief exists!!");
      return false;
    }
    dict_bel.insert({id, bel_apri});
  }

  Eigen::MatrixXd Sigma_apri = Eigen::MatrixXd::Zero(state_dim, state_dim);

  {
    // TODO: simplify to upper triangular matrix
    size_t row_start = 0;
    for (auto const &id_row : IDs) {
      size_t state_dim_row = dict_dim[id_row];
      size_t col_start = 0;
      for (auto const &id_col : IDs) {
        size_t state_dim_col = dict_dim[id_col];
        if (id_row == id_col) {
          Sigma_apri.block(row_start, col_start, state_dim_row, state_dim_col) = dict_bel[id_row]->Sigma();
        } else {
          Eigen::MatrixXd Sigma_IJ = get_Sigma_IJ_at_t(id_row, id_col, t);
          if (Sigma_IJ.size()) {
            Sigma_apri.block(row_start, col_start, state_dim_row, state_dim_col) = Sigma_IJ;
          }
        }
        col_start += state_dim_col;
      }
      row_start += state_dim_row;
    }
  }

  // stack individual's covariances:
  Sigma_apri = utils::stabilize_covariance(Sigma_apri);
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());

  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H, R, r, Sigma_apri, cfg);
  if (!res.rejected) {
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(res.Sigma_apos),
                        "Joint apos covariance is not PSD at t=" + t.str());

    // IMPORTANT: MAINTAIN ORDER STRICKTLY
    // 1) add correction terms in the appropriate correction buffers!
    {
      size_t row_start = 0;
      for (auto const &id : IDs) {
        Eigen::MatrixXd Sigma_II_apos = res.Sigma_apos.block(row_start, row_start, dict_dim[id], dict_dim[id]);
        get(id)->apply_correction_at_t(t, dict_bel[id]->Sigma(), Sigma_II_apos);
        row_start += dict_dim[id];
      }
    }

    // 2) set a corrected factorized a posterioiry cross-covariance
    // split covariance
    {
      size_t row_start = 0, col_start_offset = 0;
      for (size_t i = 0; i < IDs.size() - 1; i++) {
        size_t dim_i = dict_dim[IDs.at(i)];
        col_start_offset += dim_i;
        size_t ID_I = IDs.at(i);
        size_t col_start = col_start_offset;
        for (size_t j = i + 1; j < IDs.size(); j++) {
          size_t dim_j = dict_dim[IDs.at(j)];
          size_t ID_J = IDs.at(j);
          Eigen::MatrixXd Sigma_IJ_apos = res.Sigma_apos.block(row_start, col_start, dim_i, dim_j);
          set_Sigma_IJ_at_t(ID_I, ID_J, Sigma_IJ_apos, t);
          col_start += dim_j;
        }
        row_start += dim_i;
      }
    }

    // 3) correct beliefs implace!
    {
      size_t row_start = 0;
      for (auto const &id : IDs) {
        Eigen::MatrixXd Sigma_II_apos = res.Sigma_apos.block(row_start, row_start, dict_dim[id], dict_dim[id]);
        dict_bel[id]->correct(res.delta_mean.block(row_start, 0, dict_dim[id], 1), Sigma_II_apos);
        row_start += dict_dim[id];
      }
    }
  }
  return !res.rejected;
}

bool IsolatedKalmanFilterHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                    const Eigen::VectorXd &z, const Eigen::MatrixXd &R,
                                                    const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg) {
  // TODO:
  return false;
}

// KF: Algorithm 6 in [1]
bool IsolatedKalmanFilterHandler::apply_joint_observation(const size_t ID_I, const size_t ID_J,
                                                          const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
                                                          const Eigen::MatrixXd &R, const Eigen::VectorXd &z,
                                                          const Timestamp &t,
                                                          const KalmanFilter::CorrectionCfg_t &cfg) {
  RTV_EXPECT_TRUE_THROW(exists(ID_I) && exists(ID_J), "IKF instances do not exists!");

  // get individuals a priori beliefs:
  pBelief_t bel_I_apri, bel_J_apri;
  RTV_EXPECT_TRUE_THROW(get(ID_I)->get_belief_at_t(t, bel_I_apri), "Could not obtain belief");
  RTV_EXPECT_TRUE_THROW(get(ID_J)->get_belief_at_t(t, bel_J_apri), "Could not obtain belief");
  RTV_EXPECT_TRUE_THROW(bel_I_apri->timestamp() == bel_J_apri->timestamp(),
                        "Timestamps of bliefs need to match! Did you forgett to propagate first?");

  // stack the measurement sensitivity matrix:
  Eigen::MatrixXd H_joint = utils::horcat(H_II, H_JJ);

  // stack individual's mean:
  Eigen::VectorXd mean_joint = utils::vertcat_vec(bel_I_apri->mean(), bel_J_apri->mean());

  // residual:
  Eigen::VectorXd r = z - H_joint * mean_joint;

  return apply_joint_observation(bel_I_apri, bel_J_apri, ID_I, ID_J, H_II, H_JJ, R, r, t, cfg);
}

// EKF: Algorithm 6 in [1]
bool IsolatedKalmanFilterHandler::apply_joint_observation(pBelief_t &bel_I_apri, pBelief_t &bel_J_apri,
                                                          const size_t ID_I, const size_t ID_J,
                                                          const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
                                                          const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
                                                          const Timestamp &t,
                                                          const KalmanFilter::CorrectionCfg_t &cfg) {
  // stack the measurement sensitivity matrix (again...):
  Eigen::MatrixXd H_joint = utils::horcat(H_II, H_JJ);

  // stack individual's covariances:
  Eigen::MatrixXd Sigma_apri
    = utils::stabilize_covariance(stack_apri_covariance(bel_I_apri, bel_J_apri, ID_I, ID_J, t));
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());

  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_joint, R, r, Sigma_apri, cfg);
  if (!res.rejected) {
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(res.Sigma_apos),
                        "Joint apos covariance is not PSD at t=" + t.str());
    size_t dim_I = bel_I_apri->Sigma().rows();
    size_t dim_J = bel_J_apri->Sigma().rows();

    // split covariance
    Eigen::MatrixXd Sigma_II_apos, Sigma_JJ_apos, Sigma_IJ_apos;
    utils::split_Sigma(res.Sigma_apos, dim_I, dim_J, Sigma_II_apos, Sigma_JJ_apos, Sigma_IJ_apos);

    // IMPORTANT: keep order! before setting cross-covariance factors and beliefs implace!
    // 1) add correction terms in the appropriate correction buffers!
    get(ID_I)->apply_correction_at_t(t, bel_I_apri->Sigma(), Sigma_II_apos);
    get(ID_J)->apply_correction_at_t(t, bel_J_apri->Sigma(), Sigma_JJ_apos);

    // 2) set a corrected factorized a posterioiry cross-covariance
    set_Sigma_IJ_at_t(ID_I, ID_J, Sigma_IJ_apos, t);

    // 3) correct beliefs implace!
    bel_I_apri->correct(res.delta_mean.topLeftCorner(dim_I, 1), Sigma_II_apos);
    bel_J_apri->correct(res.delta_mean.bottomRightCorner(dim_J, 1), Sigma_JJ_apos);
  }
  return !res.rejected;
}

bool ikf::IsolatedKalmanFilterHandler::apply_joint_observation(
  pBelief_t &bel_I_apri, pBelief_t &bel_J_apri, pBelief_t &bel_K_apri, const size_t ID_I, const size_t ID_J,
  const size_t ID_K, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &H_KK,
  const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg) {
  // stack the measurement sensitivity matrix (again...):
  Eigen::MatrixXd H_joint = utils::horcat(H_II, H_JJ, H_KK);

  // stack individual's covariances:
  Eigen::MatrixXd Sigma_apri
    = utils::stabilize_covariance(stack_apri_covariance(bel_I_apri, bel_J_apri, bel_K_apri, ID_I, ID_J, ID_K, t));
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());

  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_joint, R, r, Sigma_apri, cfg);
  if (!res.rejected) {
    size_t dim_I = bel_I_apri->Sigma().rows();
    size_t dim_J = bel_J_apri->Sigma().rows();
    size_t dim_K = bel_K_apri->Sigma().rows();

    Eigen::MatrixXd Sigma_II_apos, Sigma_JJ_apos, Sigma_KK_apos;
    Eigen::MatrixXd Sigma_IJ_apos, Sigma_IK_apos, Sigma_JK_apos;
    utils::split_Sigma(res.Sigma_apos, dim_I, dim_J, dim_K, Sigma_II_apos, Sigma_JJ_apos, Sigma_KK_apos, Sigma_IJ_apos,
                       Sigma_IK_apos, Sigma_JK_apos);

    // IMPORTANT: keep order! before setting cross-covariance factors and beliefs implace!
    // 1) add correction terms in the appropriate correction buffers!
    get(ID_I)->apply_correction_at_t(t, bel_I_apri->Sigma(), Sigma_II_apos);
    get(ID_J)->apply_correction_at_t(t, bel_J_apri->Sigma(), Sigma_JJ_apos);
    get(ID_K)->apply_correction_at_t(t, bel_K_apri->Sigma(), Sigma_KK_apos);

    // 2) set a corrected factorized a posterioiry cross-covariance
    set_Sigma_IJ_at_t(ID_I, ID_J, Sigma_IJ_apos, t);
    set_Sigma_IJ_at_t(ID_I, ID_K, Sigma_IK_apos, t);
    set_Sigma_IJ_at_t(ID_J, ID_K, Sigma_JK_apos, t);

    // 3) correct beliefs implace!
    bel_I_apri->correct(res.delta_mean.topLeftCorner(dim_I, 1), Sigma_II_apos);
    bel_J_apri->correct(res.delta_mean.block(dim_I, 0, dim_J, 1), Sigma_JJ_apos);
    bel_K_apri->correct(res.delta_mean.bottomRightCorner(dim_K, 1), Sigma_KK_apos);
  }
  return !res.rejected;
}

bool IsolatedKalmanFilterHandler::apply_joint_observation(
  pBelief_t &bel_I_apri, pBelief_t &bel_J_apri, pBelief_t &bel_K_apri, pBelief_t &bel_L_apri, const size_t ID_I,
  const size_t ID_J, const size_t ID_K, const size_t ID_L, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
  const Eigen::MatrixXd &H_KK, const Eigen::MatrixXd &H_LL, const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
  const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg) {
  // RTV_EXPECT_TRUE_THROW(ID_I == m_ID, "ID_I missmatch! wrong interim master");

  // stack the measurement sensitivity matrix (again...):
  Eigen::MatrixXd H_joint = utils::horcat(H_II, H_JJ, H_KK, H_LL);

  // stack individual's covariances:
  Eigen::MatrixXd Sigma_apri = utils::stabilize_covariance(
    stack_apri_covariance(bel_I_apri, bel_J_apri, bel_K_apri, bel_L_apri, ID_I, ID_J, ID_K, ID_L, t));
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());

  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_joint, R, r, Sigma_apri, cfg);
  if (!res.rejected) {
    size_t dim_I = bel_I_apri->Sigma().rows();
    size_t dim_J = bel_J_apri->Sigma().rows();
    size_t dim_K = bel_K_apri->Sigma().rows();
    size_t dim_L = bel_L_apri->Sigma().rows();

    Eigen::MatrixXd Sigma_II_apos, Sigma_JJ_apos, Sigma_KK_apos, Sigma_LL_apos;
    ;
    Eigen::MatrixXd Sigma_IJ_apos, Sigma_IK_apos, Sigma_JK_apos;
    Eigen::MatrixXd Sigma_IL_apos, Sigma_JL_apos, Sigma_KL_apos;
    utils::split_Sigma(res.Sigma_apos, dim_I, dim_J, dim_K, dim_L, Sigma_II_apos, Sigma_JJ_apos, Sigma_KK_apos,
                       Sigma_LL_apos, Sigma_IJ_apos, Sigma_IK_apos, Sigma_JK_apos, Sigma_IL_apos, Sigma_JL_apos,
                       Sigma_KL_apos);

    // IMPORTANT: keep order! before setting cross-covariance factors and beliefs implace!
    // 1) add correction terms in the appropriate correction buffers!
    get(ID_I)->apply_correction_at_t(t, bel_I_apri->Sigma(), Sigma_II_apos);
    get(ID_J)->apply_correction_at_t(t, bel_J_apri->Sigma(), Sigma_JJ_apos);
    get(ID_K)->apply_correction_at_t(t, bel_K_apri->Sigma(), Sigma_KK_apos);
    get(ID_L)->apply_correction_at_t(t, bel_L_apri->Sigma(), Sigma_LL_apos);

    // 2) set a corrected factorized a posterioiry cross-covariance
    set_Sigma_IJ_at_t(ID_I, ID_J, Sigma_IJ_apos, t);
    set_Sigma_IJ_at_t(ID_I, ID_K, Sigma_IK_apos, t);
    set_Sigma_IJ_at_t(ID_I, ID_L, Sigma_IL_apos, t);

    set_Sigma_IJ_at_t(ID_J, ID_K, Sigma_JK_apos, t);
    set_Sigma_IJ_at_t(ID_J, ID_L, Sigma_JL_apos, t);

    set_Sigma_IJ_at_t(ID_K, ID_L, Sigma_KL_apos, t);

    // 3) correct beliefs implace!
    bel_I_apri->correct(res.delta_mean.topLeftCorner(dim_I, 1), Sigma_II_apos);
    bel_J_apri->correct(res.delta_mean.block(dim_I, 0, dim_J, 1), Sigma_JJ_apos);
    bel_K_apri->correct(res.delta_mean.block(dim_I + dim_J, 0, dim_K, 1), Sigma_KK_apos);
    bel_L_apri->correct(res.delta_mean.bottomRightCorner(dim_L, 1), Sigma_LL_apos);
  }
  return !res.rejected;
}

}  // namespace ikf
