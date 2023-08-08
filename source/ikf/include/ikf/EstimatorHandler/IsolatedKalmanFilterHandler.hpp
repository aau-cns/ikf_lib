/******************************************************************************
* FILENAME:     IsolatedKalmanFilterHandler.hpp
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
#ifndef ISOLATEDKALMANFILTERHANDLER_HPP
#define ISOLATEDKALMANFILTERHANDLER_HPP
#include <ikf/ikf_api.h>
#include <memory>
#include <ikf/Estimator/IIsolatedKalmanFilter.hpp>

namespace ikf {


class IKF_API IsolatedKalmanFilterHandler {


public:
  IsolatedKalmanFilterHandler(bool const handle_delayed = true, double const horizon_sec = 1.0);
  ~IsolatedKalmanFilterHandler() = default;

  bool add(pIKF_t p_IKF);
  pIKF_t get(const size_t ID);
  bool remove(const size_t ID);
  bool exists(const size_t ID);
  std::vector<size_t> get_instance_ids();
  double horizon_sec() const;
  void set_horizon(double const t_hor);

  void reset();

  /// \brief process_measurement: If the m_handle_delayed_meas == true, the IKF-Handler is a centralized entity hanndle
  /// all incomming measurements and is responsible to handle delayed measurements. \param m \return
  virtual ProcessMeasResult_t process_measurement(MeasData const& m);

  /// Generic fusion algorithm for M-participants:
  /// - the state dim can be infered from the cols of the H matrices
  /// - IDs of particants can be obtained through the dictionary keys.
  ///
  ///
  bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::MatrixXd& R,
                         const Eigen::VectorXd& r, const Timestamp& t, const KalmanFilter::CorrectionCfg_t& cfg);
  bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::VectorXd& z,
                         const Eigen::MatrixXd& R, const Timestamp& t, const KalmanFilter::CorrectionCfg_t& cfg);

  // KF: Algorithm 6 in [1]
  bool apply_joint_observation(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd& H_II,
                               const Eigen::MatrixXd& H_JJ, const Eigen::MatrixXd& R, const Eigen::VectorXd& z,
                               const Timestamp& t, const KalmanFilter::CorrectionCfg_t& cfg);

  // EKF: Algorithm 6 in [1]
  bool apply_joint_observation(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, const size_t ID_I, const size_t ID_J,
                               const Eigen::MatrixXd& H_II, const Eigen::MatrixXd& H_JJ, const Eigen::MatrixXd& R,
                               const Eigen::VectorXd& r, const Timestamp& t, const KalmanFilter::CorrectionCfg_t& cfg);

  bool apply_joint_observation(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, pBelief_t& bel_K_apri, const size_t ID_I,
                               const size_t ID_J, const size_t ID_K, const Eigen::MatrixXd& H_II,
                               const Eigen::MatrixXd& H_JJ, const Eigen::MatrixXd& H_KK, const Eigen::MatrixXd& R,
                               const Eigen::VectorXd& r, const Timestamp& t, const KalmanFilter::CorrectionCfg_t& cfg);

  bool apply_joint_observation(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, pBelief_t& bel_K_apri,
                               pBelief_t& bel_L_apri, const size_t ID_I, const size_t ID_J, const size_t ID_K,
                               const size_t ID_L, const Eigen::MatrixXd& H_II, const Eigen::MatrixXd& H_JJ,
                               const Eigen::MatrixXd& H_KK, const Eigen::MatrixXd& H_LL, const Eigen::MatrixXd& R,
                               const Eigen::VectorXd& r, const Timestamp& t, const KalmanFilter::CorrectionCfg_t& cfg);

protected:
  virtual bool insert_measurement(MeasData const& m, Timestamp const& t);

  void sort_measurements_from_t(Timestamp const& t);
  /////////////////////////////////////////////////////
  /// Interface for IKF handles to reprocess measurements
  TMultiHistoryBuffer<MeasData> get_measurements_from_t(Timestamp const& t);
  TMultiHistoryBuffer<MeasData> get_measurements_after_t(Timestamp const& t);
  bool is_order_violated(MeasData const& m);
  virtual bool redo_updates_from_t(const Timestamp &t);
  virtual bool redo_updates_after_t(const Timestamp &t);
  virtual ProcessMeasResult_t delegate_measurement(MeasData const& m);
  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void remove_beliefs_from_t(Timestamp const& t);

  Eigen::MatrixXd stack_apri_covariance(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, const size_t ID_I,
                                        const size_t ID_J, Timestamp const& t);

  Eigen::MatrixXd stack_apri_covariance(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, pBelief_t& bel_K_apri,
                                        const size_t ID_I, const size_t ID_J, const size_t ID_K, Timestamp const& t);
  Eigen::MatrixXd stack_apri_covariance(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, pBelief_t& bel_K_apri,
                                        pBelief_t& bel_L_apri, const size_t ID_I, const size_t ID_J, const size_t ID_K,
                                        const size_t ID_L, Timestamp const& t);

  Eigen::MatrixXd get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, Timestamp const& t);
  void set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd& Sigma_IJ, const Timestamp& t);

  Eigen::MatrixXd stack_H(const std::map<size_t, Eigen::MatrixXd>& dict_H);

  std::map<size_t, pBelief_t> get_dict_bel(const std::map<size_t, Eigen::MatrixXd>& dict_H, Timestamp const& t) {
    std::map<size_t, pBelief_t> dict_bel;
    for (auto const& e : dict_H) {
      size_t id = e.first;
      pBelief_t bel_apri;
      if (!get(id)->get_belief_at_t(t, bel_apri)) {
        RTV_EXPECT_TRUE_MSG(false, "No belief exists!!");
        return dict_bel;
      }
      dict_bel.insert({id, bel_apri});
    }
    return dict_bel;
  }

  Eigen::MatrixXd stack_Sigma_apri(const std::map<size_t, pBelief_t>& dict_bel, Timestamp const& t) {
    size_t state_dim = 0;
    for (auto const& e : dict_bel) {
      state_dim += e.second->es_dim();
    }
    Eigen::MatrixXd Sigma_apri = Eigen::MatrixXd::Zero(state_dim, state_dim);

    size_t row_start = 0;
    for (auto const& e_i : dict_bel) {
      size_t id_row = e_i.first;
      size_t state_dim_row = e_i.second->es_dim();
      size_t col_start = 0;
      for (auto const& e_j : dict_bel) {
        size_t id_col = e_j.first;
        size_t state_dim_col = e_j.second->es_dim();
        if (id_row == id_col) {
          Sigma_apri.block(row_start, col_start, state_dim_row, state_dim_col) = e_i.second->Sigma();
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

    return Sigma_apri;
  }

  void apply_corrections_at_t(Eigen::MatrixXd& Sigma_apos, std::map<size_t, size_t> const& dict_dim,
                              const std::map<size_t, pBelief_t>& dict_bel, Timestamp const& t);

  void split_right_upper_covariance(Eigen::MatrixXd& Sigma, std::map<size_t, size_t> const& dict_dim,
                                    Timestamp const& t);

  void correct_beliefs_implace(Eigen::MatrixXd& Sigma_apos, Eigen::VectorXd& delta_mean,
                               std::map<size_t, size_t> const& dict_dim, const std::map<size_t, pBelief_t>& dict_bel);

  std::unordered_map<size_t, std::shared_ptr<IIsolatedKalmanFilter>> id_dict;
  bool m_handle_delayed_meas = true;
  TTimeHorizonBuffer<MeasData, TMultiHistoryBuffer<MeasData>> HistMeas;
  double m_horzion_sec;
};  // class IsolatedKalmanFilterHandler

}  // namespace ikf

#endif // ISOLATEDKALMANFILTERHANDLER_HPP
