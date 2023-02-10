/******************************************************************************
* FILENAME:     IFilterInstance.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     25.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IFILTER_INSTANCE_HPP
#define IFILTER_INSTANCE_HPP
#include <ikf/ikf_api.h>
#include <ikf/Sensor/Estimator/ProcessMeasResult_t.hpp>
#include <ikf/Sensor/Measurement/MeasData.hpp>
#include <ikf/Sensor/Estimate/IBelief.hpp>


namespace ikf {


class IKF_API IFilterInstance {
public:
  IFilterInstance(double const horizon_sec_=1.0, bool const handle_delayed_meas=true);

  bool handle_delayed_meas() const;
  void handle_delayed_meas(bool const val);

  /// pure virtual method
  virtual ProcessMeasResult_t progapation_measurement(MeasData const& m) = 0;
  virtual ProcessMeasResult_t local_private_measurement(MeasData const& m) = 0;

  virtual void initialize(ptr_belief bel_init, Timestamp const& t);
  virtual ProcessMeasResult_t process_measurement(MeasData const& m);
  virtual bool redo_updates_after_t(Timestamp const& t);
  Timestamp current_t() const;
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

  // KF:
  virtual  bool apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);
  // EKF: if linearizing about bel_II_apri
  virtual  bool apply_propagation(ptr_belief& bel_II_apri, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);
  // migth be superfluous: virtual  bool apply_propagation(const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);

  // KF:
  virtual bool apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t);
  // EKF: if linearizing about bel_II_apri
  virtual bool apply_private_observation(ptr_belief& bel_II_apri, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t);

protected:
  TTimeHorizonBuffer<ptr_belief> HistBelief;
  TTimeHorizonBuffer<MeasData> HistMeas;
  double max_time_horizon_sec;
  bool m_handle_delayed_meas = true;  // specifies, it the instance maintains a history of past measurements or not

}; // class IFilterInstance

} // namespace ikf
#endif // IFILTER_INSTANCE_HPP
