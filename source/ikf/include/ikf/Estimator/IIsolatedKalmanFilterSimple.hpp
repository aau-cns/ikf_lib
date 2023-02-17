/******************************************************************************
* FILENAME:     IIsolatedKalmanFilterSimple.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     17.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IISOLATEDKALMANFILTERSIMPLE_HPP
#define IISOLATEDKALMANFILTERSIMPLE_HPP
#include <ikf/ikf_api.h>
#include "ikf/Estimator/IIsolatedKalmanFilter.hpp"

namespace ikf {
class IKF_API IIsolatedKalmanFilterSimple: public IIsolatedKalmanFilter {
public:
  IIsolatedKalmanFilterSimple(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, size_t const ID,
                              bool const handle_delayed_meas=true , double const horizon_sec=1.0) : IIsolatedKalmanFilter(ptr_Handler, ID, handle_delayed_meas, horizon_sec) {}
  ~IIsolatedKalmanFilterSimple() {}

  // IIsolatedKalmanFilter interface
public:
  Eigen::MatrixXd get_CrossCovFact_at_t(const Timestamp &t, size_t ID_J) override {
    Eigen::MatrixXd mat;
    auto iter = HistCrossCovFactors.find(ID_J);
    if (iter != HistCrossCovFactors.end()) {
      iter->second.get_at_t(t, mat);
    }
    return mat;
  }


protected:
  bool add_correction_at_t(const Timestamp &t_a, const Timestamp &t_b, const Eigen::MatrixXd &Phi_a_b) override {
    // apply correction to exisit cross-covariance factors at t
    for(auto & HistCCF : HistCrossCovFactors) {
      if(HistCCF.second.exist_at_t(t_a))
      {
        Eigen::MatrixXd ccf_IJ_t;
        HistCCF.second.get_at_t(t_a, ccf_IJ_t);
        HistCCF.second.insert(Phi_a_b*ccf_IJ_t, t_b);
      }
    }
    return true;
  }
  bool apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Factor) override {
    // apply correction to exisit cross-covariance factors at t
    for(auto & HistCCF : HistCrossCovFactors) {
      Timestamp latest_ccf_t;
      if(HistCCF.second.exist_at_t(t)) {
        Eigen::MatrixXd ccf_IJ_t;
        HistCCF.second.get_at_t(t, ccf_IJ_t);
        HistCCF.second.insert(Factor*ccf_IJ_t, t);
      }
    }
    return true;
  }
};


} // ns ikf
#endif // IISOLATEDKALMANFILTERSIMPLE_HPP
