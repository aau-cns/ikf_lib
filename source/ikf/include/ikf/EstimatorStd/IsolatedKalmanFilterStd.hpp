/******************************************************************************
* FILENAME:     IsolatedKalmanFilterStd.hpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
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
*  References:
*  [1] L. Luft, T. Schubert, S. I. Roumeliotis, and W. Burgard, "Recursive decentralized localization for multi-robot systems with asynchronous pairwise communication", The International Journal of Robotics Research (IJRR), 2018.
*  [2] Jung, Roland and Weiss, Stephan, "Scalable Recursive Distributed Collaborative State Estimation for Aided
Inertial Navigation", Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), IEEE, Xi’an, 2021.
*
******************************************************************************/
#ifndef ISOLATEDKALMANFILTERSTD_HPP
#define ISOLATEDKALMANFILTERSTD_HPP
#include <ikf/ikf_api.h>
#include <ikf/Estimate/IBelief.hpp>
#include <ikf/EstimatorStd/KalmanFilterStd.hpp>
namespace ikf {

class IKFHandlerStd;

///
/// \brief The IsolatedKalmanFilterStd class
/// Naive implementation of the Isolated Kalman Filter paradigm without buffering, thus no Out-Of-Sequence support.
/// Please note, as simplification, no corection buffer for correction terms as proposed in [2] is used.
///
class IKF_API IsolatedKalmanFilterStd : public KalmanFilterStd {

public:
  typedef std::shared_ptr<IKFHandlerStd> ptr_handler;

  ///
  /// \brief IsolatedKalmanFilterStd
  /// \param pHandler shared pointer to the "Instance Handler" to access participants beliefs and to perform corrections.
  /// \param belief initial belief
  /// \param ID unique ID
  ///
  IsolatedKalmanFilterStd(ptr_handler pHandler, pBelief_t const& belief, size_t const ID);
  IsolatedKalmanFilterStd(ptr_handler pHandler, size_t const ID);
  virtual ~IsolatedKalmanFilterStd();
  virtual size_t ID() const;

  // [1] Algorithm (1), [2] Algorithm (1) (without buffering).
  virtual bool propagate(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_b);

  virtual bool propagate(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_b, const Eigen::MatrixXd &G_a, const Eigen::VectorXd &u_a, const Eigen::VectorXd &var_u);

  // [1] Algorithm 2, [2] Algorithm (3) (without buffering).
  virtual bool private_update(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                      const Eigen::VectorXd &z);

  // [1] Algorithm 3, [2] Algorithm 5 (without buffering).
  bool joint_update(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
                               const Eigen::MatrixXd &R, const Eigen::VectorXd &z);

  void apply_correction(const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd &Sigma_apos);
protected:
  Eigen::MatrixXd stack_Sigma(const Eigen::MatrixXd &Sigma_II, const Eigen::MatrixXd &Sigma_JJ, const Eigen::MatrixXd &Sigma_IJ);
  void split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J, Eigen::MatrixXd& Sigma_II,
                   Eigen::MatrixXd& Sigma_JJ, Eigen::MatrixXd& Sigma_IJ);
  void set_CrossCovFact(const size_t ID_J, const Eigen::MatrixXd &SigmaFact_IJ);
  void set_Sigma_IJ(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &Sigma_IJ);

  Eigen::MatrixXd get_CrossCovFact(const size_t ID_J) const;
  Eigen::MatrixXd get_Sigma_IJ(const size_t ID_I, const size_t ID_J);
  Eigen::MatrixXd stack_apri_covariance(pBelief_t &bel_J_apri, const size_t ID_J);
  void apply_propagation(const Eigen::MatrixXd &Phi_a_b);
  void apply_correction(const Eigen::MatrixXd &Lambda);


protected:
  std::unordered_map<size_t, Eigen::MatrixXd> m_CrossCovFactors;
  size_t m_ID;
  ptr_handler m_pHandler;
};

} // ns ifk
#endif // ISOLATEDKALMANFILTERSTD_HPP
