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
#include <ikf/Estimator/KalmanFilter.hpp>
#include <ikf/Estimator/NormalizedInnovationSquared.hpp>
#include <ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp>
#include <ikf/Logger/Logger.hpp>
#include <ikf/utils/eigen_utils.hpp>

namespace ikf {

IsolatedKalmanFilterHandler::IsolatedKalmanFilterHandler(const double horizon_sec) : IDICOHandler(horizon_sec) {
  Logger::ikf_logger()->info("IsolatedKalmanFilterHandler will perform joint updates isolated ");
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
  std::vector<size_t> ids;
  for (auto const &e : dict_H) {
    ids.push_back(e.first);
  }
  return get_dict_bel(ids, t);
}

std::map<size_t, pBelief_t> ikf::IsolatedKalmanFilterHandler::get_dict_bel(const std::vector<size_t> &ids,
                                                                           const Timestamp &t) {
  std::map<size_t, pBelief_t> dict_bel;
  for (size_t id : ids) {
    pBelief_t bel_apri;
    RTV_EXPECT_TRUE_THROW(get(id)->get_belief_at_t(t, bel_apri), "Could not obtain belief");
    dict_bel.insert({id, bel_apri});
  }
  return dict_bel;
}

size_t IsolatedKalmanFilterHandler::get_dim(const std::map<size_t, pBelief_t> &dict_bel) {
  size_t dim = 0;
  for (auto const &e_i : dict_bel) {
    dim += e_i.second->es_dim();
  }
  return dim;
}

Eigen::VectorXd IsolatedKalmanFilterHandler::stack_mean(const std::map<size_t, pBelief_t> &dict_bel) {
  size_t dim = 0;
  for (auto const &e_i : dict_bel) {
    dim += e_i.second->ns_dim();
  }
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(dim, 1);

  size_t row_start = 0;
  for (auto const &e_i : dict_bel) {
    size_t state_dim_row = e_i.second->ns_dim();
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
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
  Eigen::MatrixXd H = stack_H(dict_H);

  std::map<size_t, pBelief_t> dict_bel = get_dict_bel(dict_H, t);

  Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);

  // stack individual's covariances:
  Sigma_apri = utils::stabilize_covariance(Sigma_apri);
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());

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
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
  Eigen::MatrixXd H = stack_H(dict_H);

  std::map<size_t, pBelief_t> dict_bel = get_dict_bel(dict_H, t);

  Eigen::VectorXd mean_apri = stack_mean(dict_bel);
  Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);

  Eigen::VectorXd r = z - H * mean_apri;

  // stack individual's covariances:<
  Sigma_apri = utils::stabilize_covariance(Sigma_apri);
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());

  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H, R, r, Sigma_apri, cfg);
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
}

ApplyObsResult_t IsolatedKalmanFilterHandler::apply_observation(const Eigen::MatrixXd &R, const Eigen::VectorXd &z,
                                                                const Timestamp &t,
                                                                IIsolatedKalmanFilter::h_joint const &h,
                                                                const std::vector<size_t> &IDs,
                                                                const KalmanFilter::CorrectionCfg_t &cfg) {
  std::map<size_t, pBelief_t> dict_bel;
  Eigen::MatrixXd Sigma_apos;
  Eigen::VectorXd delta_mean;

  ApplyObsResult_t res = process_observation(R, z, t, h, IDs, cfg, Sigma_apos, delta_mean, dict_bel);
  if (res.status != eMeasStatus::REJECTED) {
    // IMPORTANT: MAINTAIN ORDER STRICKTLY
    // 1) add correction terms in the appropriate correction buffers!
    apply_corrections_at_t(Sigma_apos, dict_bel, t);

    // 2) set a corrected factorized a posterioiry cross-covariance
    split_right_upper_covariance(Sigma_apos, dict_bel, t);

    // 3) correct beliefs implace!
    correct_beliefs_implace(Sigma_apos, delta_mean, dict_bel);
  }
  return res;
}

ApplyObsResult_t IsolatedKalmanFilterHandler::process_observation(
  const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t, IIsolatedKalmanFilter::h_joint const &h,
  std::vector<size_t> const &IDs, const KalmanFilter::CorrectionCfg_t &cfg, Eigen::MatrixXd &Sigma_apos,
  Eigen::VectorXd &dx, std::map<size_t, pBelief_t> &dict_bel) {
  ApplyObsResult_t res(eMeasStatus::PROCESSED);
  dict_bel = get_dict_bel(IDs, t);
  Eigen::VectorXd mean_apri = stack_mean(dict_bel);
  Eigen::MatrixXd Sigma_apri = stack_Sigma(dict_bel, t);
  // stack individual's covariances:
  Sigma_apri = utils::stabilize_covariance(Sigma_apri);
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());

  std::map<size_t, pBelief_t> bel_idx;
  size_t es_dim = 0;
  for (auto &bel_I : dict_bel) {
    bel_idx.insert({bel_I.first, bel_I.second->clone()});
    es_dim += bel_I.second->es_dim();
  }

  Eigen::MatrixXd H_idx;
  Eigen::MatrixXd K_idx;
  Eigen::VectorXd dx_idx = Eigen::VectorXd::Zero(es_dim, 1);
  Eigen::MatrixXd J_idx = Eigen::MatrixXd::Identity(es_dim, es_dim);
  Eigen::MatrixXd P_idx = Sigma_apri;

  // NOTE: iterated Error-KF update steps: we move the nominal state in each iteration, therefore the error-state
  // covariance at the nominal-state needs to be shifted as well...
  for (size_t iter = 0; iter < cfg.num_iter; iter++) {
    // compute individuals' Jacobian
    std::pair<std::map<size_t, Eigen::MatrixXd>, Eigen::VectorXd> H_r_idx = h(bel_idx, IDs, z);

    // stack Jacobian
    H_idx = stack_H(H_r_idx.first);

    // compute residual
    Eigen::VectorXd r_idx = H_r_idx.second;

    if (!KalmanFilter::check_dim(H_idx, R, r_idx, Sigma_apri)) {
      return ApplyObsResult_t(eMeasStatus::DISCARED);
    }
    Eigen::VectorXd e_x_idx = Eigen::VectorXd::Zero(es_dim, 1);
    if (iter > 0) {
      // TODO: we need the box-minus operator here per belief!

      size_t row_start = 0;
      for (auto &bel_apri_i : dict_bel) {
        size_t dim_I = bel_apri_i.second->es_dim();

        // e_idx = (x_est_idx boxminus x_apri)
        // Eigen::VectorXd e_x_ii = bel_i.second->boxminus(bel_idx.at(bel_i.first));

        // https://github.com/decargroup/navlie/blob/a19dbe32cc5337048ca2a20f67852150b2322513/navlie/filters.py#L360
        // bel_err_idx = wedge(bel_apri^(inv) * bel_idx);  bel_err_idx = bel_idx - bel_apri;
        Eigen::VectorXd e_x_ii = bel_idx.at(bel_apri_i.first)->boxminus(bel_apri_i.second);
        // Eigen::VectorXd e_x_ii = bel_apri_i.second->boxminus(bel_idx.at(bel_apri_i.first));

        e_x_idx.block(row_start, 0, dim_I, 1) = e_x_ii;
        row_start += dim_I;
      }

      // stack state Jacobian_plus
      row_start = 0;
      for (auto &bel_idx_i : bel_idx) {
        size_t dim_I = bel_idx_i.second->es_dim();

        // https://github.com/decargroup/navlie/blob/a19dbe32cc5337048ca2a20f67852150b2322513/navlie/filters.py#L361
        // We need the right Jacobian
        J_idx.block(row_start, row_start, dim_I, dim_I)
          = bel_idx_i.second->plus_jacobian(-e_x_idx.block(row_start, 0, dim_I, 1));
        row_start += dim_I;
      }

      // r_idx = z_est -  H_idx*(x_est_apri - x_est_idx)
      r_idx = r_idx + H_idx * (J_idx * e_x_idx);
    }

    P_idx = J_idx * Sigma_apri * J_idx.transpose();

    Eigen::MatrixXd S_idx = H_idx * P_idx * H_idx.transpose() + R;
    S_idx = utils::stabilize_covariance(S_idx, cfg.eps);
    if (cfg.use_outlier_rejection) {
      if (!NormalizedInnovationSquared::check_NIS(S_idx, r_idx, cfg.confidence_interval)) {
        // outlier...
        return ApplyObsResult_t(eMeasStatus::REJECTED, r_idx);
      }
    }
    res.residual = r_idx;

    K_idx = P_idx * H_idx.transpose() * S_idx.inverse();

    // mean correction
    dx_idx = -J_idx * e_x_idx + K_idx * r_idx;

    size_t row_start = 0;
    for (auto const &bel_i : bel_idx) {
      size_t dim_I = bel_i.second->es_dim();
      bel_i.second->boxplus(dx_idx.block(row_start, 0, dim_I, 1));
      row_start += dim_I;
    }

    // check the difference of the means over the iteration steps:
    if (dx_idx.norm() < cfg.tol_eps && iter > 0) {
      // ikf::Logger::ikf_logger()->warn("Stopped after n iterations:" + std::to_string(iter));
      break;
    }
  }

  Eigen::MatrixXd U = (Eigen::MatrixXd::Identity(es_dim, es_dim) - K_idx * H_idx);
  if (cfg.use_Josephs_form) {
    Sigma_apos = U * P_idx * U.transpose() + K_idx * R * K_idx.transpose();
  } else {
    Sigma_apos = U * P_idx;
  }

  if (cfg.nummerical_stabilization) {
    Sigma_apos = utils::stabilize_covariance(Sigma_apos, cfg.eps);
  }

  bool is_psd = utils::is_positive_semidefinite(Sigma_apos);
  RTV_EXPECT_TRUE_MSG(is_psd, "Joint apos covariance is not PSD at t=" + t.str());
  if (!is_psd) {
    Sigma_apos = utils::nearest_covariance(Sigma_apos, 1e-6);
  }

  // correct inplace:
  dx = Eigen::VectorXd::Zero(es_dim, 1);
  for (auto const &bel_i : bel_idx) {
    dict_bel.at(bel_i.first)->mean(bel_i.second->mean());
  }
  return res;
}

}  // namespace ikf
