/******************************************************************************
* FILENAME:     IKalmanFilter.hpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         <roland.jung@ieee.org>
* VERSION:      v0.0.0
* CREATION:     25.01.2023
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
#ifndef I_KALMAN_FILTER_HPP
#define I_KALMAN_FILTER_HPP
#include <ikf/ikf_api.h>
#include "ikf/Container/TMultiHistoryBuffer.hpp"
#include <ikf/Estimator/KalmanFilter.hpp>
#include <ikf/Estimator/ProcessMeasResult_t.hpp>
#include <ikf/Measurement/MeasData.hpp>
#include <ikf/Estimate/IBelief.hpp>


namespace ikf {

enum class IKF_API eGetBeliefStrategy {
  EXACT = 0,    // a belief is expected at a given timestamp
  CLOSEST = 1,  // if exist, return the closes belief in HistBelief (if any)
  LINEAR_INTERPOL_BELIEF
  = 2,  // if exist, interpolate between two beliefs and add them to the HistBelief, else PREDICT_BELIEF
  LINEAR_INTERPOL_MEAS
  = 3,  // if exist, interpolate between proprioceptive measurements linearly and perform a pseudo prediction step.
  PREDICT_BELIEF = 4,  // if KF has a prediction model, a belief is predicted
  AUTO = 5,            // 1.) exact, 2.) if(proagation measurements available) 3). linear interpol meas, ele predict.
};

std::string IKF_API to_string(const eGetBeliefStrategy e);
eGetBeliefStrategy IKF_API str2eGetBeliefStrategy(const std::string &str);

///
/// \brief The IKalmanFilter class
/// Supporting delayed/ out-of-order/sequence measurements. In order to reprocess measurements, they need to be
/// stored as "MeasData" in a fixed time horizon buffer "HistMeas".
class IKF_API IKalmanFilter {
public:
  IKalmanFilter(double const horizon_sec_=1.0, bool const handle_delayed_meas=true);
  IKalmanFilter(pBelief_t bel_0, double const horizon_sec_=1.0, bool const handle_delayed_meas=true);

  ///////////////////////////////////////////////////////////////////////////////////
  /// Trigger the filter:
  virtual ProcessMeasResult_vec_t process_measurement(MeasData const &m);
  ///////////////////////////////////////////////////////////////////////////////////

  bool handle_delayed_meas() const;
  void handle_delayed_meas(bool const val);
  bool enabled() const;
  void enabled(bool const val);
  virtual void initialize(pBelief_t bel_init);
  virtual void initialize(pBelief_t bel_init, Timestamp const &t);
  virtual void set_horizon(double const t_hor);
  virtual void reset();
  Timestamp current_t() const;
  pBelief_t current_belief() const;
  bool exist_belief_at_t(Timestamp const &t) const;

  pBelief_t get_belief_at_t(Timestamp const &t) const;
  pBelief_t get_belief_at_t(Timestamp const &t, eGetBeliefStrategy const type);
  bool get_belief_at_t(Timestamp const &t, pBelief_t &bel, eGetBeliefStrategy const type = eGetBeliefStrategy::EXACT);
  void set_belief_at_t(pBelief_t const &bel, Timestamp const &t);
  bool get_belief_before_t(Timestamp const &t, pBelief_t &bel, Timestamp &t_before);
  Eigen::VectorXd get_mean_at_t(Timestamp const &t) const;
  Eigen::MatrixXd get_Sigma_at_t(Timestamp const &t) const;
  void print_HistMeas(size_t max = 100, bool reverse = false);
  void print_HistBelief(size_t max = 100, bool reverse = false);

  bool get_prop_meas_at_t(Timestamp const &t, MeasData &m);

protected:
  ///////////////////////////////////////////////////////////////////////////////////
  /// pure virtual method
  virtual bool predict_to(const ikf::Timestamp &t_b) = 0; // predict from a previous belief to t_b using the stochastic system's model (in case there is one)
  virtual ProcessMeasResult_t progapation_measurement(MeasData const& m) = 0;
  virtual ProcessMeasResult_t local_private_measurement(MeasData const& m) = 0;
  /// pure virtual method
  ///////////////////////////////////////////////////////////////////////////////////

  virtual bool insert_measurement(MeasData const &m, Timestamp const &t);

  virtual ProcessMeasResult_vec_t redo_updates_after_t(Timestamp const &t);
  bool correct_belief_at_t(Eigen::VectorXd const& mean_corr, Eigen::MatrixXd const& Sigma_apos, Timestamp const&t);

  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void check_horizon();

  ///
  /// \brief delegate_measurement: redirects measurement either to progapation_measurement() or
  /// local_private_measurement() based on the measurement's eObservationType
  /// \param m
  /// \return ProcessMeasResult_t
  ///
  virtual ProcessMeasResult_t delegate_measurement(MeasData const &m);

  // KF:
  virtual  bool apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);
  // EKF: if linearizing about bel_II_apri
  virtual  bool apply_propagation(pBelief_t& bel_II_a, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);

  virtual  bool apply_propagation(pBelief_t bel_II_b, const Eigen::MatrixXd &Phi_II_ab,  const Timestamp &t_a, const Timestamp &t_b);


  // KF:
  virtual bool apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg);
  // EKF: if linearizing about bel_II_apri
  virtual bool apply_private_observation(pBelief_t &bel_II_apri, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                         const Eigen::VectorXd &r, const KalmanFilter::CorrectionCfg_t &cfg);

  TTimeHorizonBuffer<pBelief_t> HistBelief;
  TTimeHorizonBuffer<MeasData, TMultiHistoryBuffer<MeasData>> HistMeas;
  TTimeHorizonBuffer<MeasData> HistMeasPropagation;
  double max_time_horizon_sec;
  bool m_handle_delayed_meas = true;  // specifies, if the instance maintains a history of past measurements or not
  bool m_enabled = true;              // specifies, if the instance processed measurements or not
  KalmanFilter::CorrectionCfg_t m_CorrCfg; // specifies the default correction config.


}; // class IKalmanFilter

} // namespace ikf
#endif // I_KALMAN_FILTER_HPP
