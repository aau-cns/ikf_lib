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

enum class eGetBeliefStrategy {
  EXACT = 0,   // a belief is expected at a given timestamp
  CLOSEST = 1,  // if exist, return the closes belief in HistBelief (if any)
  LINEAR_INTERPOL_BELIEF = 2, // if exist, interpolate between two beliefs and add them to the HistBelief, else PREDICT_BELIEF
  LINEAR_INTERPOL_MEAS = 3, // if exist, interpolate between proprioceptive measurements linearly and perform a pseudo prediction step.
  PREDICT_BELIEF = 4, // if KF has a prediction model, a belief is predicted
};

///
/// \brief The IKalmanFilter class
/// Supporting delayed/ out-of-order/sequence measurements. In order to reprocess measurements, they need to be
/// stored as "MeasData" in a fixed time horizon buffer "HistMeas".
class IKF_API IKalmanFilter {
public:
  IKalmanFilter(double const horizon_sec_=1.0, bool const handle_delayed_meas=true);
  IKalmanFilter(ptr_belief bel_0, double const horizon_sec_=1.0, bool const handle_delayed_meas=true);

  ///////////////////////////////////////////////////////////////////////////////////
  /// Trigger the filter:
  virtual ProcessMeasResult_t process_measurement(MeasData const& m);
  ///////////////////////////////////////////////////////////////////////////////////

  bool handle_delayed_meas() const;
  void handle_delayed_meas(bool const val);
  virtual void initialize(ptr_belief bel_init);
  virtual void initialize(ptr_belief bel_init, Timestamp const& t);
  virtual void set_horizon(double const t_hor);
  virtual void reset();
  virtual bool insert_measurement(MeasData const& m, Timestamp const& t);

  virtual bool redo_updates_after_t(Timestamp const& t);
  Timestamp current_t() const;
  ptr_belief current_belief() const;
  bool exist_belief_at_t(Timestamp const& t) const;

  ptr_belief get_belief_at_t(Timestamp const& t) const;
  ptr_belief get_belief_at_t(Timestamp const& t, eGetBeliefStrategy const type);
  bool get_belief_at_t(Timestamp const& t, ptr_belief& bel, eGetBeliefStrategy const type=eGetBeliefStrategy::EXACT);
  void set_belief_at_t(ptr_belief const& bel, Timestamp const&t);
  bool get_belief_before_t(Timestamp const&t, ptr_belief& bel, Timestamp &t_before);
  Eigen::VectorXd get_mean_at_t(Timestamp const &t) const;
  Eigen::MatrixXd get_Sigma_at_t(Timestamp const&t) const;
  void print_HistMeas(size_t max=100, bool reverse=false);
  void print_HistBelief(size_t max=100, bool reverse=false);

protected:
  ///////////////////////////////////////////////////////////////////////////////////
  /// pure virtual method
  virtual bool predict_to(const ikf::Timestamp &t_b) = 0; // predict from a previous belief to t_b using the stochastic system's model (in case there is one)
  virtual ProcessMeasResult_t progapation_measurement(MeasData const& m) = 0;
  virtual ProcessMeasResult_t local_private_measurement(MeasData const& m) = 0;
  /// pure virtual method
  ///////////////////////////////////////////////////////////////////////////////////
  bool correct_belief_at_t(Eigen::VectorXd const& mean_corr, Eigen::MatrixXd const& Sigma_apos, Timestamp const&t);

  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void check_horizon();

  virtual ProcessMeasResult_t reprocess_measurement(MeasData const& m);
  // KF:
  virtual  bool apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);
  // EKF: if linearizing about bel_II_apri
  virtual  bool apply_propagation(ptr_belief& bel_II_a, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);

  virtual  bool apply_propagation(ptr_belief bel_II_b, const Eigen::MatrixXd &Phi_II_ab,  const Timestamp &t_a, const Timestamp &t_b);


  // KF:
  virtual bool apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg);
  // EKF: if linearizing about bel_II_apri
  virtual bool apply_private_observation(ptr_belief& bel_II_apri, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const KalmanFilter::CorrectionCfg_t &cfg);


  TTimeHorizonBuffer<ptr_belief> HistBelief;
  TTimeHorizonBuffer<MeasData, TMultiHistoryBuffer<MeasData>> HistMeas;
  TTimeHorizonBuffer<MeasData> HistMeasPropagation;
  double max_time_horizon_sec;
  bool m_handle_delayed_meas = true;  // specifies, if the instance maintains a history of past measurements or not
  KalmanFilter::CorrectionCfg_t m_CorrCfg; // specifies the default correction config.


}; // class IKalmanFilter

} // namespace ikf
#endif // I_KALMAN_FILTER_HPP
