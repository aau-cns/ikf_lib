/******************************************************************************
* FILENAME:     IKalmanFilter.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     25.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef I_KALMAN_FILTER_HPP
#define I_KALMAN_FILTER_HPP
#include "ikf/Container/TMultiHistoryBuffer.hpp"
#include <ikf/ikf_api.h>
#include <ikf/Estimator/ProcessMeasResult_t.hpp>
#include <ikf/Measurement/MeasData.hpp>
#include <ikf/Estimate/IBelief.hpp>


namespace ikf {

///
/// \brief The IKalmanFilter class
/// Supporting delayed/ out-of-order/sequence measurements. In order to reprocess measurements, they need to be
/// stored as "MeasData" in a fixed time horizon buffer "HistMeas".
class IKF_API IKalmanFilter {
public:
  IKalmanFilter(double const horizon_sec_=1.0, bool const handle_delayed_meas=true);
  IKalmanFilter(ptr_belief bel_0, double const horizon_sec_=1.0, bool const handle_delayed_meas=true);
  ///////////////////////////////////////////////////////////////////////////////////
  /// pure virtual method
  virtual ProcessMeasResult_t progapation_measurement(MeasData const& m) = 0;
  virtual ProcessMeasResult_t local_private_measurement(MeasData const& m) = 0;
  /// pure virtual method
  ///////////////////////////////////////////////////////////////////////////////////
  ///
  virtual ProcessMeasResult_t process_measurement(MeasData const& m);
  ///
  ///////////////////////////////////////////////////////////////////////////////////

  bool handle_delayed_meas() const;
  void handle_delayed_meas(bool const val);
  virtual void initialize(ptr_belief bel_init);
  virtual void initialize(ptr_belief bel_init, Timestamp const& t);
  virtual bool redo_updates_after_t(Timestamp const& t);
  Timestamp current_t() const;
  ptr_belief current_belief() const;
  bool exist_belief_at_t(Timestamp const& t) const;
  ptr_belief get_belief_at_t(Timestamp const& t) const;
  bool get_belief_at_t(Timestamp const& t, ptr_belief& bel);
  void set_belief_at_t(ptr_belief const& bel, Timestamp const&t);
  bool correct_belief_at_t(Eigen::VectorXd const& mean_corr, Eigen::MatrixXd const& Sigma_apos, Timestamp const&t);
  bool get_belief_before_t(Timestamp const&t, ptr_belief& bel, Timestamp &t_before);
  Eigen::VectorXd get_mean_at_t(Timestamp const &t) const;
  Eigen::MatrixXd get_Sigma_at_t(Timestamp const&t) const;
  virtual void reset();
  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void set_horizon(double const t_hor);
  virtual void check_horizon();

  void print_HistMeas(size_t max=100, bool reverse=false);
  void print_HistBelief(size_t max=100, bool reverse=false);
protected:
  virtual ProcessMeasResult_t reprocess_measurement(MeasData const& m);
  virtual bool propagate_from_to(const Timestamp &t_a, const Timestamp &t_b);

  // KF:
  virtual  bool apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);
  // EKF: if linearizing about bel_II_apri
  virtual  bool apply_propagation(ptr_belief& bel_II_apri, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);
  // migth be superfluous: virtual  bool apply_propagation(const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);

  // KF:
  virtual bool apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t);
  // EKF: if linearizing about bel_II_apri
  virtual bool apply_private_observation(ptr_belief& bel_II_apri, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &r);


  TTimeHorizonBuffer<ptr_belief> HistBelief;
  TTimeHorizonBuffer<MeasData, TMultiHistoryBuffer<MeasData>> HistMeas;
  TTimeHorizonBuffer<MeasData> HistMeasPropagation;
  double max_time_horizon_sec;
  bool m_handle_delayed_meas = true;  // specifies, if the instance maintains a history of past measurements or not

}; // class IKalmanFilter

} // namespace ikf
#endif // I_KALMAN_FILTER_HPP
