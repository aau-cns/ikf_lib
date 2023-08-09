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

bool IsolatedKalmanFilterHandler::exists(const size_t ID) { return (id_dict.find(ID) != id_dict.end()); }

std::vector<size_t> IsolatedKalmanFilterHandler::get_instance_ids() {
  std::vector<size_t> IDs;
  IDs.reserve(id_dict.size());
  for (auto const &elem : id_dict) {
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
  if (HistMeas.get_latest_t(t_latest)) {
    TMultiHistoryBuffer<MeasData> meas = HistMeas.get_between_t1_t2(t, t_latest);

    if (!meas.empty()) {
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
  ProcessMeasResult_t res = delegate_measurement(m);
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

ProcessMeasResult_t IsolatedKalmanFilterHandler::delegate_measurement(const MeasData &m) {
  ProcessMeasResult_t res;
  if (exists(m.id_sensor)) {
    res = id_dict[m.id_sensor]->delegate_measurement(m);
  }
  return res;
}

bool IsolatedKalmanFilterHandler::redo_updates_from_t(const Timestamp &t) {
  remove_beliefs_from_t(t);
  Timestamp t_last;
  if (HistMeas.get_latest_t(t_last)) {
    Logger::ikf_logger()->info("IsolatedKalmanFilterHandler::redo_updates_from_t() t=" + t.str()
                               + ", t_last=" + t_last.str());

    if (t == t_last) {
      auto vec = HistMeas.get_all_at_t(t);
      for (MeasData &m : vec) {
        this->delegate_measurement(m);
      }
    } else {
      HistMeas.foreach_between_t1_t2(t, t_last, [this](MeasData const &m) { this->delegate_measurement(m); });
    }
    return true;
  }
  return false;
}

bool IsolatedKalmanFilterHandler::redo_updates_after_t(const Timestamp &t) {
  remove_beliefs_after_t(t);
  Timestamp t_after, t_last;
  if (HistMeas.get_after_t(t, t_after) && HistMeas.get_latest_t(t_last)) {
    Logger::ikf_logger()->info("IsolatedKalmanFilterHandler::redo_updates_after_t() t_after=" + t_after.str()
                               + ", t_last=" + t_last.str());
    if (t_after == t_last) {
      auto vec = HistMeas.get_all_at_t(t_after);
      for (MeasData &m : vec) {
        this->delegate_measurement(m);
      }
    } else {
      HistMeas.foreach_between_t1_t2(t_after, t_last, [this](MeasData const &m) { this->delegate_measurement(m); });
    }
    return true;
  }
  return false;
}

void IsolatedKalmanFilterHandler::remove_beliefs_after_t(const Timestamp &t) {
  for (auto &elem : id_dict) {
    elem.second->remove_after_t(t);
  }
}

void IsolatedKalmanFilterHandler::remove_beliefs_from_t(const Timestamp &t) {
  for (auto &elem : id_dict) {
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
  for (auto &elem : id_dict) {
    elem.second->reset();
  }
}

bool IsolatedKalmanFilterHandler::is_order_violated(const MeasData &m) {
  if (m.obs_type != eObservationType::JOINT_OBSERVATION) {
    auto meas_arr = HistMeas.get_all_at_t(m.t_m);

    if (m.obs_type == eObservationType::PROPAGATION) {
      for (MeasData &m_ : meas_arr) {
        if (m_.obs_type == eObservationType::PRIVATE_OBSERVATION
            || m_.obs_type == eObservationType::JOINT_OBSERVATION) {
          return true;
        }
      }
    } else if (m.obs_type == eObservationType::PRIVATE_OBSERVATION) {
      for (MeasData &m_ : meas_arr) {
        if (m_.obs_type == eObservationType::JOINT_OBSERVATION) {
          return true;
        }
      }
    }
  }
  return false;
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

Eigen::MatrixXd IsolatedKalmanFilterHandler::stack_H(const std::map<size_t, Eigen::MatrixXd> &dict_H) {
  size_t state_dim = 0;
  size_t H_rows = 0;
  for (auto const &e : dict_H) {
    state_dim += e.second.cols();
    H_rows = e.second.rows();
  }

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(H_rows, state_dim);
  {
    size_t col_start = 0;
    for (auto const &e : dict_H) {
      size_t dim_I = e.second.cols();
      H.block(0, col_start, H_rows, dim_I) = e.second;
      col_start += dim_I;
    }
  }
  return H;
}

std::map<size_t, pBelief_t> IsolatedKalmanFilterHandler::get_dict_bel(const std::map<size_t, Eigen::MatrixXd> &dict_H,
                                                                      const Timestamp &t) {
  std::map<size_t, pBelief_t> dict_bel;
  for (auto const &e : dict_H) {
    size_t id = e.first;
    pBelief_t bel_apri;
    RTV_EXPECT_TRUE_THROW(get(id)->get_belief_at_t(t, bel_apri), "Could not obtain belief");
    dict_bel.insert({id, bel_apri});
  }
  return dict_bel;
}

Eigen::VectorXd IsolatedKalmanFilterHandler::stack_mean(const std::map<size_t, pBelief_t> &dict_bel) {
  size_t state_dim = 0;
  for (auto const &e : dict_bel) {
    state_dim += e.second->es_dim();
  }
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(state_dim, 1);

  size_t row_start = 0;
  for (auto const &e_i : dict_bel) {
    size_t state_dim_row = e_i.second->es_dim();
    mean.block(row_start, 0, state_dim_row, 1) = e_i.second->mean();
    row_start += state_dim_row;
  }
  return mean;
}

// Algorithm 6 in [1]
Eigen::MatrixXd IsolatedKalmanFilterHandler::stack_Sigma(const std::map<size_t, pBelief_t> &dict_bel,
                                                         const Timestamp &t) {
  size_t state_dim = 0;
  for (auto const &e : dict_bel) {
    state_dim += e.second->es_dim();
  }
  Eigen::MatrixXd Sigma = Eigen::MatrixXd::Zero(state_dim, state_dim);

  size_t row_start = 0;
  for (auto const &e_i : dict_bel) {
    size_t id_row = e_i.first;
    size_t state_dim_row = e_i.second->es_dim();
    size_t col_start = 0;
    for (auto const &e_j : dict_bel) {
      size_t id_col = e_j.first;
      size_t state_dim_col = e_j.second->es_dim();
      if (id_row == id_col) {
        Sigma.block(row_start, col_start, state_dim_row, state_dim_col) = e_i.second->Sigma();
      } else {
        // obtain only the upper triangular part (if dict_bel was sorted ascending)
        if (id_row < id_col) {
          Eigen::MatrixXd Sigma_IJ = get_Sigma_IJ_at_t(id_row, id_col, t);
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

void IsolatedKalmanFilterHandler::apply_corrections_at_t(Eigen::MatrixXd &Sigma_apos,
                                                         const std::map<size_t, pBelief_t> &dict_bel,
                                                         const Timestamp &t) {
  size_t row_start = 0;
  for (auto const &e_i : dict_bel) {
    size_t dim_I = e_i.second->es_dim();

    Eigen::MatrixXd Sigma_II_apos = Sigma_apos.block(row_start, row_start, dim_I, dim_I);
    get(e_i.first)->apply_correction_at_t(t, e_i.second->Sigma(), Sigma_II_apos);
    row_start += dim_I;
  }
}

void IsolatedKalmanFilterHandler::split_right_upper_covariance(Eigen::MatrixXd &Sigma,
                                                               const std::map<size_t, pBelief_t> &dict_bel,
                                                               const Timestamp &t) {
  std::vector<size_t> IDs;
  for (auto const &e : dict_bel) {
    IDs.push_back(e.first);
  }

  size_t row_start = 0, col_start_offset = 0;
  for (size_t i = 0; i < IDs.size() - 1; i++) {
    size_t dim_i = dict_bel.at(IDs.at(i))->es_dim();
    col_start_offset += dim_i;
    size_t ID_I = IDs.at(i);
    size_t col_start = col_start_offset;
    for (size_t j = i + 1; j < IDs.size(); j++) {
      size_t dim_j = dict_bel.at(IDs.at(j))->es_dim();
      size_t ID_J = IDs.at(j);
      Eigen::MatrixXd Sigma_IJ_apos = Sigma.block(row_start, col_start, dim_i, dim_j);
      set_Sigma_IJ_at_t(ID_I, ID_J, Sigma_IJ_apos, t);
      col_start += dim_j;
    }
    row_start += dim_i;
  }
}

void IsolatedKalmanFilterHandler::correct_beliefs_implace(Eigen::MatrixXd &Sigma_apos, Eigen::VectorXd &delta_mean,
                                                          std::map<size_t, pBelief_t> const &dict_bel) {
  size_t row_start = 0;
  for (auto const &e_i : dict_bel) {
    size_t dim_I = e_i.second->es_dim();

    Eigen::MatrixXd Sigma_II_apos = Sigma_apos.block(row_start, row_start, dim_I, dim_I);
    e_i.second->correct(delta_mean.block(row_start, 0, dim_I, 1), Sigma_II_apos);
    row_start += dim_I;
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
  }
  return !res.rejected;
}

bool IsolatedKalmanFilterHandler::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H,
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

    // IMPORTANT: MAINTAIN ORDER STRICKTLY
    // 1) add correction terms in the appropriate correction buffers!
    apply_corrections_at_t(res.Sigma_apos, dict_bel, t);

    // 2) set a corrected factorized a posterioiry cross-covariance
    split_right_upper_covariance(res.Sigma_apos, dict_bel, t);

    // 3) correct beliefs implace!
    correct_beliefs_implace(res.Sigma_apos, res.delta_mean, dict_bel);
  }
  return !res.rejected;
}

}  // namespace ikf
