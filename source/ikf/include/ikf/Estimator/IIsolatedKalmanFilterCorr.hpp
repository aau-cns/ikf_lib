/******************************************************************************
* FILENAME:     IIsolatedKalmanFilterCorr.hpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         <roland.jung@aau.at>
* VERSION:      v0.0.1
* CREATION:     18.02.2023
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
#ifndef IISOLATEDKALMANFILTERCORR_HPP
#define IISOLATEDKALMANFILTERCORR_HPP
#include <ikf/ikf_api.h>
#include "ikf/Estimator/IIsolatedKalmanFilter.hpp"

namespace ikf {

class IKF_API IIsolatedKalmanFilterCorr: public IIsolatedKalmanFilter {
public:
  IIsolatedKalmanFilterCorr(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, size_t const ID, bool const handle_delayed_meas=true , double const horizon_sec=1.0);
  ~IIsolatedKalmanFilterCorr();

  void reset() override;

  Eigen::MatrixXd get_CrossCovFact_at_t(const Timestamp &t, size_t ID_J) override;

  virtual void remove_after_t(const Timestamp &t) override;

  virtual void set_horizon(const double t_hor) override;

  // Algorithm 3 in [1]
  void check_correction_horizon();

  void check_horizon() override;

  // Algorithm 1 in [1]
  Eigen::MatrixXd compute_correction(const Timestamp &t_a, const Timestamp &t_b) const;

  // Eq. 8 in [1]
  bool add_correction_at_t(const Timestamp &t_a, const Timestamp &t_b, const Eigen::MatrixXd &Phi_a_b) override;

  // Eq. 15, 21 in [1]
  bool apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Factor) override;

  // Eq. 20 in [1]
  bool apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd Sigma_apos) override;

  void print_HistCorr(size_t max, bool reverse);

protected:
  TTimeHorizonBuffer<Eigen::MatrixXd> HistCorr; //  TimeHorizonBuffer<Corrections>; Corrections := {Phi, Lambda Epsilon}
};

}


#endif // IISOLATEDKALMANFILTERCORR_HPP
