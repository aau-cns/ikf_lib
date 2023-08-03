/******************************************************************************
* FILENAME:     IsolatedKalmanFilter.hpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         <roland.jung@aau.at>
* VERSION:      v0.0.0
* CREATION:     30.01.2023
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
*
*
*  References:
*  [1] Jung, Roland and Weiss, Stephan, "Modular Multi-Sensor Fusion: A Collaborative State Estimation Perspective", IEEE Robotics and Automation Letters, DOI: 10.1109/LRA.2021.3096165, 2021.
******************************************************************************/
#ifndef IISOLATEDKALMANFILTER_HPP
#define IISOLATEDKALMANFILTER_HPP

#include <ikf/ikf_api.h>
#include <ikf/Container/Timestamp.hpp>
#include <ikf/Container/TTimeHorizonBuffer.hpp>
#include <ikf/Estimator/KalmanFilter.hpp>
#include <ikf/Estimator/IKalmanFilter.hpp>

namespace ikf {

class IsolatedKalmanFilterHandler;


///
/// \brief The IsolatedKalmanFilter class: A modular and decoupled sensor fusion strategy by Roland Jung and Stephan Weiss.
///
class IKF_API IIsolatedKalmanFilter: public IKalmanFilter {
public:
  IIsolatedKalmanFilter(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, size_t const ID,
                        bool const handle_delayed_meas=true , double const horizon_sec=1.0);
  virtual ~IIsolatedKalmanFilter() {}

  virtual size_t ID() const;
  void reset() override;
  virtual void initialize(pBelief_t bel_init) override;
  virtual void initialize(pBelief_t bel_init, Timestamp const& t) override;
  virtual void set_horizon(double const t_hor) override;

  virtual bool insert_measurement(MeasData const& m, Timestamp const& t) override;

  ///////////////////////////////////////////////////////////////////////////////////
  /// Trigger the filter:
  // Algorithm 7 in [1]
  virtual ProcessMeasResult_t process_measurement(MeasData const& m) override;
  ///////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  /// inter-filter interface:
  TMultiHistoryBuffer<MeasData> get_measurements_after_t(Timestamp const&t);
  TMultiHistoryBuffer<MeasData> get_measurements_from_t(Timestamp const&t);
  std::vector<MeasData> get_measurements_at_t(Timestamp const&t) {
    return HistMeas.get_all_at_t(t);
  }

    // Algorithm 7 in [1]
  virtual ProcessMeasResult_t reprocess_measurement(MeasData const& m) override;

  virtual Eigen::MatrixXd get_CrossCovFact_at_t(Timestamp const& t, size_t ID_J);
  void set_CrossCovFact_at_t(Timestamp const& t, size_t const unique_ID, Eigen::MatrixXd const& ccf);
  // Eq. 20 in [1]
  virtual bool apply_correction_at_t(Timestamp const&t, Eigen::MatrixXd const& Sigma_apri, Eigen::MatrixXd const Sigma_apos);
  ///////////////////////////////////////////////////////////////////////////////////
  virtual void remove_after_t(Timestamp const& t);
  virtual void remove_from_t(Timestamp const& t);

protected:
  bool is_order_violated(MeasData const& m);
  virtual bool redo_updates_after_t(Timestamp const& t) override;
  ///////////////////////////////////////////////////////////////////////////////////
  /// pure virtual method
  virtual ProcessMeasResult_t local_joint_measurement(MeasData const& m) = 0;
  /// pure virtual method
  ///////////////////////////////////////////////////////////////////////////////////

  bool get_CrossCovFact_at_t(Timestamp const& t, size_t ID_J, Eigen::MatrixXd &FFC);

  Eigen::MatrixXd get_CrossCovFact_before_t(Timestamp const& t, size_t unique_ID) const;

  void propagate_CrossCovFact(Timestamp const& t_a, Timestamp const& t_b, Eigen::MatrixXd const& M_a_b);


  // Algorithm 3 in [1]
  virtual void check_horizon() override;

  // Eq. 8 in [1]
  virtual bool add_correction_at_t(const Timestamp &t_a,Timestamp const& t_b, Eigen::MatrixXd const& Phi_a_b);
  // Eq. 15, 21 in [1]
  virtual bool apply_correction_at_t(Timestamp const&t, Eigen::MatrixXd const& Factor);

  // Algorithm 7 in [1]
  //virtual ProcessMeasResult_t reprocess_measurement(MeasData const& m);
  ///////////////////////////////////////////
  /// FUSION LOGIC:
  // KF: Algorithm 8 in [1]
  bool apply_propagation(const Eigen::MatrixXd& Phi_II_ab, const Eigen::MatrixXd& Q_II_ab, const Timestamp& t_a,
                         const Timestamp& t_b) override;
  // EKF: Algorithm 8 in [1]
  bool apply_propagation(pBelief_t& bel_II_apri, const Eigen::VectorXd& mean_II_b, const Eigen::MatrixXd& Phi_II_ab,
                         const Eigen::MatrixXd& Q_II_ab, const Timestamp& t_a, const Timestamp& t_b) override;

  virtual bool apply_propagation(pBelief_t bel_II_b, const Eigen::MatrixXd& Phi_II_ab, const Timestamp& t_a,
                                 const Timestamp& t_b) override;

  // KF:  Algorithm 4 in [1]
  bool apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg) override;
  // EKF: Algorithm 4 in [1]
  bool apply_private_observation(pBelief_t& bel_II_apri, const Eigen::MatrixXd& H_II, const Eigen::MatrixXd& R,
                                 const Eigen::VectorXd& r, const Timestamp& t,
                                 const KalmanFilter::CorrectionCfg_t& cfg);

  bool apply_private_observation(pBelief_t& bel_II_apri, const size_t ID_I, const Eigen::MatrixXd& H_II,
                                 const Eigen::MatrixXd& R, const Eigen::VectorXd& r, const Timestamp& t,
                                 const KalmanFilter::CorrectionCfg_t& cfg);

  // KF: Algorithm 6 in [1]
  bool apply_joint_observation(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
                               const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t, const KalmanFilter::CorrectionCfg_t& cfg);

  // EKF: Algorithm 6 in [1]
  bool apply_joint_observation(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, const size_t ID_I, const size_t ID_J,
                               const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &R,
                               const Eigen::VectorXd &r, const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg);


  bool apply_joint_observation(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, pBelief_t& bel_K_apri,
                               const size_t ID_I, const size_t ID_J, const size_t ID_K,
                               const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &H_KK,
                               const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
                               const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg);

  bool apply_joint_observation(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri,
                               pBelief_t& bel_K_apri, pBelief_t& bel_L_apri,
                               const size_t ID_I, const size_t ID_J, const size_t ID_K, const size_t ID_L,
                               const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
                               const Eigen::MatrixXd &H_KK, const Eigen::MatrixXd &H_LL,
                               const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
                               const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg);

  static Eigen::MatrixXd stack_Sigma(const Eigen::MatrixXd &Sigma_II, const Eigen::MatrixXd &Sigma_JJ, const Eigen::MatrixXd &Sigma_IJ);
  static void split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J, Eigen::MatrixXd& Sigma_II, Eigen::MatrixXd& Sigma_JJ, Eigen::MatrixXd& Sigma_IJ);

    static void split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J, size_t const dim_K,
                          Eigen::MatrixXd& Sigma_II, Eigen::MatrixXd& Sigma_JJ, Eigen::MatrixXd& Sigma_KK,
                          Eigen::MatrixXd& Sigma_IJ, Eigen::MatrixXd& Sigma_IK, Eigen::MatrixXd& Sigma_JK);

  static void split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J, size_t const dim_K, size_t const dim_L,
                          Eigen::MatrixXd& Sigma_II, Eigen::MatrixXd& Sigma_JJ, Eigen::MatrixXd& Sigma_KK, Eigen::MatrixXd& Sigma_LL,
                          Eigen::MatrixXd& Sigma_IJ, Eigen::MatrixXd& Sigma_IK, Eigen::MatrixXd& Sigma_JK,
                            Eigen::MatrixXd& Sigma_IL, Eigen::MatrixXd& Sigma_JL, Eigen::MatrixXd& Sigma_KL);

  Eigen::MatrixXd stack_apri_covariance(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, const size_t ID_I, const size_t ID_J,
                                        Timestamp const& t);

  Eigen::MatrixXd stack_apri_covariance(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, pBelief_t& bel_K_apri,
                                        const size_t ID_I, const size_t ID_J, const size_t ID_K,
                                        Timestamp const& t);
  Eigen::MatrixXd stack_apri_covariance(pBelief_t& bel_I_apri, pBelief_t& bel_J_apri, pBelief_t& bel_K_apri, pBelief_t& bel_L_apri,
                                        const size_t ID_I, const size_t ID_J, const size_t ID_K, const size_t ID_L,
                                        Timestamp const& t);


  Eigen::MatrixXd get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, Timestamp const& t);
  void set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &Sigma_IJ, const Timestamp &t);

  std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler;
  std::unordered_map<size_t, TTimeHorizonBuffer<Eigen::MatrixXd>> HistCrossCovFactors;
  size_t m_ID;


}; // class IIsolatedKalmanFilter

typedef std::shared_ptr<IIsolatedKalmanFilter> pIKF_t;

}


#endif // IISOLATEDKALMANFILTER_HPP
