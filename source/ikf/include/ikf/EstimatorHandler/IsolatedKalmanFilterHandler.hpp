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
  IsolatedKalmanFilterHandler(bool const handle_delayed=true, double const horizon_sec=1.0);
  ~IsolatedKalmanFilterHandler() = default;

  bool add(pIKF_t p_IKF);
  pIKF_t get(const size_t ID);
  bool remove(const size_t ID);
  bool exists(const size_t ID);
  std::vector<size_t> get_instance_ids();
  double horizon_sec() const;

  ///
  /// \brief process_measurement: If the m_handle_delayed_meas == true, the IKF-Handler is a centralized entity hanndle
  /// all incomming measurements and is responsible to handle delayed measurements. \param m \return
  virtual ProcessMeasResult_t process_measurement(MeasData const& m);

  void reset();

  /// Generic fusion algorithm for M-participants:
  /// - the state dim can be obtained through the cols of the H matrices
  /// - IDs of particants can be obtained through the dictionary keys.
  bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::MatrixXd& R,
                         const Eigen::VectorXd& r, const Timestamp& t, const KalmanFilter::CorrectionCfg_t& cfg);

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
  virtual ProcessMeasResult_t reprocess_measurement(MeasData const& m);
  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void remove_beliefs_from_t(Timestamp const& t);

  static Eigen::MatrixXd stack_Sigma(const Eigen::MatrixXd& Sigma_II, const Eigen::MatrixXd& Sigma_JJ,
                                     const Eigen::MatrixXd& Sigma_IJ);
  static void split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J,
                          Eigen::MatrixXd& Sigma_II, Eigen::MatrixXd& Sigma_JJ, Eigen::MatrixXd& Sigma_IJ);

  static void split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J, size_t const dim_K,
                          Eigen::MatrixXd& Sigma_II, Eigen::MatrixXd& Sigma_JJ, Eigen::MatrixXd& Sigma_KK,
                          Eigen::MatrixXd& Sigma_IJ, Eigen::MatrixXd& Sigma_IK, Eigen::MatrixXd& Sigma_JK);

  static void split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J, size_t const dim_K,
                          size_t const dim_L, Eigen::MatrixXd& Sigma_II, Eigen::MatrixXd& Sigma_JJ,
                          Eigen::MatrixXd& Sigma_KK, Eigen::MatrixXd& Sigma_LL, Eigen::MatrixXd& Sigma_IJ,
                          Eigen::MatrixXd& Sigma_IK, Eigen::MatrixXd& Sigma_JK, Eigen::MatrixXd& Sigma_IL,
                          Eigen::MatrixXd& Sigma_JL, Eigen::MatrixXd& Sigma_KL);

  Eigen::MatrixXd stack_apri_covariance(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, const size_t ID_I,
                                        const size_t ID_J, Timestamp const& t);

  Eigen::MatrixXd stack_apri_covariance(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, pBelief_t& bel_K_apri,
                                        const size_t ID_I, const size_t ID_J, const size_t ID_K, Timestamp const& t);
  Eigen::MatrixXd stack_apri_covariance(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, pBelief_t& bel_K_apri,
                                        pBelief_t& bel_L_apri, const size_t ID_I, const size_t ID_J, const size_t ID_K,
                                        const size_t ID_L, Timestamp const& t);

  Eigen::MatrixXd get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, Timestamp const& t);
  void set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd& Sigma_IJ, const Timestamp& t);

  std::unordered_map<size_t, std::shared_ptr<IIsolatedKalmanFilter>> id_dict;
  bool m_handle_delayed_meas = true;
  TTimeHorizonBuffer<MeasData, TMultiHistoryBuffer<MeasData>> HistMeas;
  double m_horzion_sec;
};


} // ns mmsf

#endif // ISOLATEDKALMANFILTERHANDLER_HPP
