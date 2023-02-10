/******************************************************************************
* FILENAME:     MMSF_IKF.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef MMSF_IKF_HPP
#define MMSF_IKF_HPP
#include <ikf/ikf_api.h>
#include <ikf/Strategies/IIKF_DICO.hpp>

namespace ikf {

class IKF_API MMSF_IKF: public IIKF_DICO {
public:
  MMSF_IKF(std::shared_ptr<IInstanceHandler> ptr_hdl, std::string const &name, size_t ID = 0, double const horizon_sec=1.0);

  void reset();

  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void set_horizon(double const t_hor);

  // -- check_horizon(): Algorithm 2 in [1]
  virtual void check_horizon();

  // -- compute_correction(): Algorithm 4 in [1]
  Eigen::MatrixXd compute_correction(Timestamp const& t_a, Timestamp const& t_b);

  // -- apply_correction_at_t(): Eq. 7 in [1]
  bool add_correction_at_t(Timestamp const& t_b, Eigen::MatrixXd const& Phi_a_b);

  // -- apply_correction_at_t(): Eq. 14, 15 in [1]
  bool apply_correction_at_t(Timestamp const&t, Eigen::MatrixXd const& Factor);

  // -- apply_correction_at_t(): Eq. 14, 15 in [1]
  bool apply_correction_at_t(Timestamp const&t, Eigen::MatrixXd const& Sigma_apri, Eigen::MatrixXd const Sigma_apos);


  // IMMSF interface:
  bool apply_propagation(const size_t ID_I, const Eigen::VectorXd &mean_II, const Eigen::MatrixXd &Phi_II,
                         const Eigen::MatrixXd &Q_II, const Timestamp &t_old, const Timestamp &t_new);
  bool apply_private_observation(const size_t ID_I, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                 const Eigen::VectorXd &r, const Timestamp &t);
  bool apply_joint_observation(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
                               const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t);
protected:
  TTimeHorizonBuffer<Eigen::MatrixXd> HistCorr; // % TimeHorizonBuffer<Corrections>; Corrections := {Phi, Lambda Epsilon}



}; // class MMSF_IKF


} // namespace ikf

#endif // MMSF_IKF_HPP
