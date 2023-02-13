/******************************************************************************
* FILENAME:     IsolatedKalmanFilter.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     30.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IISOLATEDKALMANFILTER_HPP
#define IISOLATEDKALMANFILTER_HPP

#include <ikf/ikf_api.h>
#include <ikf/Container/Timestamp.hpp>
#include <ikf/Container/TTimeHorizonBuffer.hpp>
#include <ikf/Estimator/IKalmanFilter.hpp>

namespace ikf {

class IsolatedKalmanFilterHandler;


///
/// \brief The IsolatedKalmanFilter class: A modular sensor fusion strategy by Roland Jung and Stephan Weiss.
///
class IKF_API IIsolatedKalmanFilter: public IKalmanFilter {
public:
  IIsolatedKalmanFilter(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, std::string const &name, size_t const ID,
                        bool const handle_delayed_meas=true , double const horizon_sec=1.0);
  virtual ~IIsolatedKalmanFilter() {}

  virtual size_t ID() const { return m_ID; }
  virtual std::string name() const { return m_name; }
  virtual bool enabled() const { return m_enabled; }
  virtual void enabled(bool const val) { m_enabled = val; }
  void reset();
  /// pure virtual method
  virtual ProcessMeasResult_t local_joint_measurement(MeasData const& m) = 0;
  virtual ProcessMeasResult_t process_measurement(MeasData const& m);

  virtual void initialize(ptr_belief bel_init, Timestamp const& t);

  virtual bool get_apri_belief_and_fcc_at_t(Timestamp const& t, size_t ID_other,  ptr_belief ptr_bel, Eigen::MatrixXd & factorized_cross_cov);
  virtual bool set_apos_belief_and_fcc_at_t(Timestamp const& t, size_t ID_other, ptr_belief ptr_bel, Eigen::MatrixXd & factorized_cross_cov);
 virtual bool set_apos_belief_and_fcc_at_t(Timestamp const& t, size_t ID_other, Eigen::VectorXd const& mean_corr,
                                            Eigen::MatrixXd const& Sigma,  Eigen::MatrixXd const& factorized_cross_cov);

  virtual bool redo_updates_after_t(Timestamp const& t);
  virtual bool clone_fccs(size_t const ID_old, size_t const ID_new);
  virtual std::vector<size_t> get_correlated_IDs() const;


  bool get_CrossCovFact_at_t(Timestamp const& t, size_t unique_ID, Eigen::MatrixXd &FFC);
  Eigen::MatrixXd get_CrossCovFact_at_t(Timestamp const& t, size_t unique_ID);
  Eigen::MatrixXd get_CrossCovFact_before_t(Timestamp const& t, size_t unique_ID) const;
  void set_CrossCovFact_at_t(Timestamp const& t, size_t const unique_ID, Eigen::MatrixXd const& ccf);
  void propagate_CrossCovFact(Timestamp const& t_a, Timestamp const& t_b, Eigen::MatrixXd const& M_a_b);
  virtual void remove_after_t(Timestamp const& t);
  virtual void set_horizon(double const t_hor);
  // -- check_horizon(): Algorithm 2 in [1]
  virtual void check_horizon();
  // -- compute_correction(): Algorithm 4 in [1]
  Eigen::MatrixXd compute_correction(Timestamp const& t_a, Timestamp const& t_b) const;
  // -- apply_correction_at_t(): Eq. 7 in [1]
  bool add_correction_at_t(Timestamp const& t_b, Eigen::MatrixXd const& Phi_a_b);
  // -- apply_correction_at_t(): Eq. 14, 15 in [1]
  bool apply_correction_at_t(Timestamp const&t, Eigen::MatrixXd const& Factor);
  // -- apply_correction_at_t(): Eq. 14, 15 in [1]
  bool apply_correction_at_t(Timestamp const&t, Eigen::MatrixXd const& Sigma_apri, Eigen::MatrixXd const Sigma_apos);

protected:
  ///////////////////////////////////////////
  /// FUSION LOGIC:
  // KF:
  bool apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);
  // EKF:
  bool apply_propagation(ptr_belief& bel_II_apri, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab,
                         const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b);

  // KF:
  bool apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t);
  // EKF:
  bool apply_private_observation(ptr_belief& bel_II_apri,const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                 const Eigen::VectorXd &r, const Timestamp &t);

  bool apply_joint_observation(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
                               const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t);
  bool apply_joint_observation(ptr_belief& bel_II_apri, ptr_belief& bel_JJ_apri, const size_t ID_I, const size_t ID_J,
                               const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &R,
                               const Eigen::VectorXd &r, const Timestamp &t);

  //Eigen::MatrixXd stack_apri_covariance(const size_t ID_I, const size_t ID_J, Timestamp const& t);
  Eigen::MatrixXd stack_apri_covariance(ptr_belief& bel_I_apri, ptr_belief& bel_J_apri, const size_t ID_I, const size_t ID_J,
                                        Timestamp const& t);
  Eigen::MatrixXd get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, Timestamp const& t);
  void set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &Sigma_IJ, const Timestamp &t);

  std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler;
  std::unordered_map<size_t, TTimeHorizonBuffer<Eigen::MatrixXd>> HistCrossCovFactors;
  TTimeHorizonBuffer<Eigen::MatrixXd> HistCorr; // % TimeHorizonBuffer<Corrections>; Corrections := {Phi, Lambda Epsilon}
  size_t keep_n_elems = 4;
  std::string m_name = "unknown";
  size_t m_ID;
  bool m_enabled = true;


}; // class DCSE_DAH


}


#endif // IISOLATEDKALMANFILTER_HPP
