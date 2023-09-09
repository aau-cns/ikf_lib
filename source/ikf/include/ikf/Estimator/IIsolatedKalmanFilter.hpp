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

class IDICOHandler;

///
/// \brief The IsolatedKalmanFilter class: A modular and decoupled sensor fusion strategy by Roland Jung and Stephan
/// Weiss.
/// - The IsolatedKalmanFilter supports a state decoupling strategy, which requires in update steps only IKF instances
/// that are output-coupled.
/// - If the IKF instance has no direct access to other instances via pointers, it is necessary/recommended to use the
/// IKF_Handler as fusion entity - as it has per definition access to all local instances. Local in this context referes
/// to objects/instances within a common process. Having a (centralized) fusion entity, does not mitigate/harm the
/// benefits of the state decoupling regarding scalabilty and modularity. The fusion center underlines/emphasises the
/// responsibilties. It is also an oppurtonity to implement and use different decoupling strategies by replacing the FC
/// and the handler to the filter intance.
/// - The local fusion center (IDICOHandler) is responsible for
/// --- stacking the beliefs
/// --- stacking measurement matrices
/// --- to call the KF update step (outlier rejection)
/// --- splitting the apos covariance
/// --- correcting the means
/// --- applying the correction terms
/// --- factorize the cross-covariances and to update the previous ones.
/// - The collaborative Fusion Center needs to exchange information between other Fusion Centers (overwrites methods of
/// local FC)
/// --- optinally maintaining the measurements (is recommended to simplify the algorithm).
///
/// - The responsibilities of the IIsolatedKalmanFilter realization:
/// --- to define which state definition is used.
/// --- to compute the Jacobians of the propagtion model and the measurment models.
/// --- to define which IKF instances are involved in the update and to compute the measurmenet matrix accordingly
/// --- dispatching/interpreting the measurement
/// --- to initialize the estimator
/// --- to perform propagation and private update steps w/o FC.
/// --- to call the FC methods in case of joint updates or a private update with FC if the ego belief is fixed!
/// --- provide an interface for the FC to get/set_FCC and to apply_a_correction()
///
/// Imagine the following example: 2 instances on two agents. The local instances can exchange information locally via
/// their handler and direct access the memory. While inter-agent coupling required information exchange. The handler
/// knows if the instance is local or remote, thus it can request data from other handlers, right? This logic can be
/// implement in either the filter instance or the handler. While on the handler side it would be clearer - while on the
/// other hand, it might be confusing why the IKF is an extension of the KF, if it requires a central handler... ->
/// Answer: The IKF-Paradigm is a state decoupling strategy. Meaning, it allows partitioning full-state vector in
/// smaller junks, if their input/prediction model is decoupled and to couple their outputs through their fusion center.
///
class IKF_API IIsolatedKalmanFilter: public IKalmanFilter {
public:
  IIsolatedKalmanFilter(std::shared_ptr<IDICOHandler> ptr_Handler, size_t const ID, double const horizon_sec = 1.0);
  virtual ~IIsolatedKalmanFilter() {}

  size_t ID() const;
  void reset() override;
  virtual void set_horizon(double const t_hor) override;

  ///////////////////////////////////////////////////////////////////////////////////
  /// Trigger the filter:
  // Algorithm 7 in [1]
  virtual ProcessMeasResult_vec_t process_measurement(MeasData const& m) override;
  ///////////////////////////////////////////////////////////////////////////////////

  virtual Eigen::MatrixXd get_CrossCovFact_at_t(Timestamp const& t, size_t ID_J);
  void set_CrossCovFact_at_t(Timestamp const& t, size_t const unique_ID, Eigen::MatrixXd const& ccf);
  // Eq. 20 in [1]
  virtual bool apply_correction_at_t(Timestamp const& t, Eigen::MatrixXd const& Sigma_apri,
                                     Eigen::MatrixXd const Sigma_apos);
  // Eq. 15, 21 in [1]
  virtual bool apply_correction_at_t(Timestamp const& t, Eigen::MatrixXd const& Factor);

  // change the fusion strategy at runtime (make sure that the new DICOHandler has the measurement buffer copied):
  virtual bool set_DICOHandler(std::shared_ptr<IDICOHandler> DICOHandler_ptr);

protected:
  /// FRIEND:
  friend IDICOHandler;
  ///////////////////////////////////////////////////////////////////////////////////
  /// IDICOHandler interface:
  ///
  /// \brief delegate_measurement: redirects measurement either to progapation_measurement(),
  /// local_private_measurement(), or local_joint_measurement() based on the measurement's eObservationType
  /// \param m
  /// \return ProcessMeasResult_t
  virtual ProcessMeasResult_t delegate_measurement(MeasData const& m) override;

  virtual void remove_after_t(Timestamp const& t);
  virtual void remove_from_t(Timestamp const& t);
  /// IDICOHandler interface:
  ///////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  /// pure virtual method
  virtual ProcessMeasResult_t local_joint_measurement(MeasData const& m) = 0;
  /// pure virtual method
  ///////////////////////////////////////////////////////////////////////////////////

  // HOOK for IKalmanFilter
  virtual bool insert_measurement(MeasData const& m, Timestamp const& t) override;

  virtual ProcessMeasResult_vec_t redo_updates_after_t(Timestamp const& t) override;

  bool get_CrossCovFact_at_t(Timestamp const& t, size_t ID_J, Eigen::MatrixXd& FFC);

  Eigen::MatrixXd get_CrossCovFact_before_t(Timestamp const& t, size_t unique_ID) const;

  void propagate_CrossCovFact(Timestamp const& t_a, Timestamp const& t_b, Eigen::MatrixXd const& M_a_b);

  // Algorithm 3 in [1]
  virtual void check_horizon() override;

  // Eq. 8 in [1]
  virtual bool add_correction_at_t(const Timestamp &t_a,Timestamp const& t_b, Eigen::MatrixXd const& Phi_a_b);

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
                                 const Eigen::VectorXd& r, const KalmanFilter::CorrectionCfg_t& cfg) override;

  bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::MatrixXd& R,
                         const Eigen::VectorXd& r, const ikf::Timestamp& t,
                         const ikf::KalmanFilter::CorrectionCfg_t& cfg);

  // call m_pHandler->apply_observation(...)
  //  bool apply_private_observation(pBelief_t& bel_II_apri, const size_t ID_I, const Eigen::MatrixXd& H_II,
  //                                 const Eigen::MatrixXd& R, const Eigen::VectorXd& r,
  //                                 const KalmanFilter::CorrectionCfg_t& cfg);

  std::shared_ptr<IDICOHandler> m_pHandler;
  std::unordered_map<size_t, TTimeHorizonBuffer<Eigen::MatrixXd>> HistCrossCovFactors;
  size_t m_ID;


}; // class IIsolatedKalmanFilter

typedef std::shared_ptr<IIsolatedKalmanFilter> pIKF_t;

}


#endif // IISOLATEDKALMANFILTER_HPP
