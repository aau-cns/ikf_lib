/******************************************************************************
* FILENAME:     IIsolatedKalmanFilterCorr.cpp
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
#include <ikf/Estimator/IIsolatedKalmanFilterCorr.hpp>
#include <ikf/Logger/Logger.hpp>

namespace  ikf {

IIsolatedKalmanFilterCorr::IIsolatedKalmanFilterCorr(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, const size_t ID, const bool handle_delayed_meas, const double horizon_sec) : IIsolatedKalmanFilter(ptr_Handler, ID, handle_delayed_meas, horizon_sec), HistCorr(horizon_sec) {}

IIsolatedKalmanFilterCorr::~IIsolatedKalmanFilterCorr() {}

void IIsolatedKalmanFilterCorr::reset() {
  IIsolatedKalmanFilter::reset();
  HistCorr.clear();
}

Eigen::MatrixXd IIsolatedKalmanFilterCorr::get_CrossCovFact_at_t(const Timestamp &t, size_t ID_J) {
  Eigen::MatrixXd mat;
  auto iter = HistCrossCovFactors.find(ID_J);
  if (iter != HistCrossCovFactors.end()) {
    bool res = iter->second.get_at_t(t, mat);
    if (!res) {

      // forward propagate FCC from previous to current timestamp and insert it in the buffer!
      Timestamp t_prev;
      res = iter->second.get_before_t(t, t_prev);
      if (res) {
        res = iter->second.get_at_t(t_prev, mat);
        Eigen::MatrixXd M_a_b = compute_correction(t_prev, t);
        if (res && M_a_b.size() > 0 ) {
          mat = M_a_b * mat;
          set_CrossCovFact_at_t(t, ID_J, mat);
        }
        else {
          Logger::ikf_logger()->info("IKF::get_CrossCovFact_at_t(): could not compute correction between t_prev=" + t_prev.str() + " and t_curr=" +  t.str());
        }
      }
      else {
        Logger::ikf_logger()->info("IKF::get_CrossCovFact_at_t(): could not find elem for id=" + std::to_string(ID_J) +" at t="      + t.str());
        Logger::ikf_logger()->info("IKF::get_CrossCovFact_at_t(): could not find elem for id=" + std::to_string(ID_J) +" at t_prev=" + t_prev.str());
      }
    }
  }


  return mat;
}

void IIsolatedKalmanFilterCorr::remove_after_t(const Timestamp &t) {
  IIsolatedKalmanFilter::remove_after_t(t);
  HistCorr.remove_after_t(t);
}

void IIsolatedKalmanFilterCorr::remove_from_t(const Timestamp &t) {
  IIsolatedKalmanFilter::remove_from_t(t);
  HistCorr.remove_after_t(t);
  HistCorr.remove_at_t(t);
}

void IIsolatedKalmanFilterCorr::set_horizon(const double t_hor) {
  IIsolatedKalmanFilter::set_horizon(t_hor);
  HistCorr.set_horizon(t_hor*2);

}

void IIsolatedKalmanFilterCorr::check_correction_horizon() {
  double cur_horizon = HistCorr.horizon();
  double half_horizon = HistCorr.max_horizon()*0.5;
  if  (cur_horizon > half_horizon) {
    // if the latest element of FCCs is about to fall outside the correction buffer's horizon
    // propagate it to the middle of the buffer.
    Timestamp oldest_t;
    HistCorr.get_oldest_t(oldest_t);

    Timestamp middle_t;
    HistCorr.get_before_t(Timestamp(oldest_t.to_sec() + half_horizon), middle_t);

    for(auto & HistCCF : HistCrossCovFactors) {
      Timestamp latest_ccf_t;
      if(HistCCF.second.get_latest_t(latest_ccf_t)) {
        if(latest_ccf_t <= middle_t) {
          RTV_EXPECT_TRUE_THROW(HistCorr.exist_at_t(latest_ccf_t), "No correction term found at t=" + latest_ccf_t.str());
          Eigen::MatrixXd M_latest_to_mid = compute_correction(latest_ccf_t, middle_t);
          Eigen::MatrixXd CCF_latest;
          HistCCF.second.get_latest(CCF_latest);
          HistCCF.second.insert(M_latest_to_mid * CCF_latest, middle_t);
        }
      }
    }
  }
}

void IIsolatedKalmanFilterCorr::check_horizon() {
  // IMPORTANT: before the horizon is about to be shrinked!
  check_correction_horizon();
  HistCorr.check_horizon();
  IIsolatedKalmanFilter::check_horizon();
}

Eigen::MatrixXd IIsolatedKalmanFilterCorr::compute_correction(const Timestamp &t_a, const Timestamp &t_b) const {
  TStampedData<Eigen::MatrixXd> data_after_ta, data_tb;

  bool exist_after_ta = HistCorr.get_after_t(t_a, data_after_ta);
  bool exist_at_tb = HistCorr.get_at_t(t_b, data_tb.data);

  Timestamp t_after_a = data_after_ta.stamp;
  Timestamp t_b_found = t_b;

  if (!exist_at_tb) {
    exist_at_tb= HistCorr.get_before_t(t_b, data_tb);
    t_b_found = data_tb.stamp;
  }

  if (exist_after_ta && exist_at_tb) {
    int max_dim = std::max(data_after_ta.data.cols(), data_after_ta.data.rows());
    Eigen::MatrixXd I  = Eigen::MatrixXd::Identity(max_dim, max_dim);
    Eigen::MatrixXd M_a_b = HistCorr.accumulate_between_t1_t2(t_after_a, t_b_found, I,
                                                              [](Eigen::MatrixXd const&A, Eigen::MatrixXd const&B){
                                                                return B*A;
                                                              });
    return M_a_b;
  }
  else {
    Logger::ikf_logger()->warn("IKF::compute_correction(): no element found for timestamps:[" + t_a.str() + "," + t_b_found.str() + "]");
  }
  return Eigen::MatrixXd();
}

bool IIsolatedKalmanFilterCorr::add_correction_at_t(const Timestamp &t_a, const Timestamp &t_b, const Eigen::MatrixXd &Phi_a_b) {
  bool res = RTV_EXPECT_TRUE_MSG(!HistCorr.exist_at_t(t_b), "correction term from t_a" + t_a.str() +"  already exists at t_b:" + t_b.str() + "! First propagate, then update!");
  if (res){
    HistCorr.insert(Phi_a_b, t_b);
  }
  else {
    // print recent correction terms:
    print_HistCorr(10, true);
  }
  return res;
}

bool IIsolatedKalmanFilterCorr::apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Factor) {
  RTV_EXPECT_TRUE_MSG(Factor.cols() == Factor.rows(), "Factor must be a square matrix!");

  // apply correction on exisit cross-covariance factors at t
  for(auto & HistCCF : HistCrossCovFactors) {
    Timestamp latest_ccf_t;
    if(HistCCF.second.exist_at_t(t)) {
      Eigen::MatrixXd ccf_IJ_t;
      HistCCF.second.get_at_t(t, ccf_IJ_t);
      HistCCF.second.insert(Factor*ccf_IJ_t, t);
    }
  }

  // store the correction factor for dated ccfs
  Eigen::MatrixXd mat;
  if (HistCorr.get_at_t(t, mat)) {
    mat = Factor * mat;
    HistCorr.insert(mat, t);
    return true;
  }
  else {
    HistCorr.insert(Factor, t);
    Logger::ikf_logger()->info("IKF::apply_correction_at_t(): no element found for timestamps:[" + t.str() + "]");
    return false;
  }
}

bool IIsolatedKalmanFilterCorr::apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd Sigma_apos) {
  Eigen::MatrixXd Lambda = Sigma_apos * Sigma_apri.inverse();
  return apply_correction_at_t(t, Lambda);
}

void IIsolatedKalmanFilterCorr::print_HistCorr(size_t max, bool reverse) {
  size_t cnt = 0;
  auto lambda = [&cnt, max](Eigen::MatrixXd const& i){
    if(cnt < max) {
      std::stringstream ss;
      ss << i;
      Logger::ikf_logger()->trace("* " + ss.str());;
    }
    cnt++;
  };
  if (!reverse) {
    HistCorr.foreach(lambda);
  } else {
    HistCorr.foreach_reverse(lambda);
  }
}

} // ns ikf
