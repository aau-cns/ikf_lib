/******************************************************************************
* FILENAME:     IsolatedKalmanFilter.cpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     03.02.2023
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
*  [1] Jung, Roland and Weiss, Stephan, "Modular Multi-Sensor Fusion: A Collaborative State Estimation Perspective", IEEE RA-L, DOI: 10.1109/LRA.2021.3096165, 2021.
******************************************************************************/
#include <ikf/Estimator/IIsolatedKalmanFilter.hpp>
#include <ikf/Estimator/KalmanFilter.hpp>
#include <ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp>
#include <ikf/Logger/Logger.hpp>
#include <ikf/utils/eigen_utils.hpp>

namespace ikf {

IIsolatedKalmanFilter::IIsolatedKalmanFilter(std::shared_ptr<IDICOHandler> ptr_Handler, const size_t ID,
                                             const double horizon_sec, const std::string &type)
  : IKalmanFilter(horizon_sec, false), m_pHandler(ptr_Handler), m_ID(ID), m_type(type) {
  Logger::ikf_logger()->info("IIsolatedKalmanFilter: horizon_sec=" + std::to_string(horizon_sec) + ", ID=["
                             + std::to_string(ID) + "], type=" + type);
}

size_t IIsolatedKalmanFilter::ID() const { return m_ID; }


void IIsolatedKalmanFilter::reset() {
  IKalmanFilter::reset();
  HistCrossCovFactors.clear();
}

// Algorithm 7 in [1]
ProcessMeasResult_vec_t IIsolatedKalmanFilter::process_measurement(const MeasData &m) {
  return m_pHandler->process_measurement(m);
}

ProcessMeasResult_vec_t IIsolatedKalmanFilter::redo_updates_after_t(const Timestamp &t) {
  remove_after_t(t);
  return IKalmanFilter::redo_updates_after_t(t);
}

bool IIsolatedKalmanFilter::get_CrossCovFact_at_t(const Timestamp &t, size_t ID_J, Eigen::MatrixXd &FFC){
  FFC = get_CrossCovFact_at_t(t, ID_J);
  return FFC.size() > 0;
}

Eigen::MatrixXd IIsolatedKalmanFilter::get_CrossCovFact_at_t(const Timestamp &t, size_t ID_J) {
  Eigen::MatrixXd mat;
  auto iter = HistCrossCovFactors.find(ID_J);
  if (iter != HistCrossCovFactors.end()) {
    iter->second.get_at_t(t, mat);
  }
  return mat;
}

Eigen::MatrixXd IIsolatedKalmanFilter::get_CrossCovFact_before_t(const Timestamp &t, size_t unique_ID) const{
  Eigen::MatrixXd mat;
  auto iter = HistCrossCovFactors.find(unique_ID);
  if (iter != HistCrossCovFactors.end()) {
    bool res = iter->second.get_before_t(t, mat);
    if (!res) {
      Logger::ikf_logger()->info("IMMSF.get_CrossCovFact_before_t(): could not find elem for id=" + std::to_string(unique_ID) + " at t=" + t.str());
    }
  }
  return mat;
}

void IIsolatedKalmanFilter::set_CrossCovFact_at_t(const Timestamp &t, const size_t unique_ID, const Eigen::MatrixXd &ccf) {
  auto iter = HistCrossCovFactors.find(unique_ID);
  if (iter != HistCrossCovFactors.end()) {
    iter->second.insert(ccf, t);
  } else {
    auto iter = HistCrossCovFactors.emplace(unique_ID, TTimeHorizonBuffer<Eigen::MatrixXd>(max_time_horizon_sec));
    iter.first->second.insert(ccf, t);
  }
}

void IIsolatedKalmanFilter::propagate_CrossCovFact(const Timestamp &t_a, const Timestamp &t_b, const Eigen::MatrixXd &M_a_b) {
  for (auto &elem  : HistCrossCovFactors) {
    Eigen::MatrixXd CrossCovFact_ta;
    if (elem.second.get_at_t(t_a, CrossCovFact_ta)) {
      Eigen::MatrixXd CrossCovFact_tb = M_a_b * CrossCovFact_ta;
      elem.second.insert(CrossCovFact_tb, t_b);
    }
  }
}


void IIsolatedKalmanFilter::remove_after_t(const Timestamp &t) {
  IKalmanFilter::remove_beliefs_after_t(t);
  for (auto& elem : HistCrossCovFactors){
    elem.second.remove_after_t(t);
  }
}

void IIsolatedKalmanFilter::remove_from_t(const Timestamp &t) {
  HistBelief.remove_after_t(t);
  HistBelief.remove_at_t(t);
  for (auto& elem : HistCrossCovFactors){
    elem.second.remove_after_t(t);
    elem.second.remove_at_t(t);
  }
}

void IIsolatedKalmanFilter::set_horizon(const double t_hor) {
  IKalmanFilter::set_horizon(t_hor);
  for (auto& elem : HistCrossCovFactors){
    elem.second.set_horizon(t_hor);
  }
}

void IIsolatedKalmanFilter::check_horizon() {
  size_t const keep_elems = 4;

  HistBelief.check_horizon_restricted(keep_elems);
  HistMeas.check_horizon_restricted(keep_elems);
  HistMeasPropagation.check_horizon_restricted(keep_elems);
  for (auto& elem : HistCrossCovFactors){
    elem.second.check_horizon_restricted(keep_elems);
  }
}


bool IIsolatedKalmanFilter::add_correction_at_t(const Timestamp &t_a, const Timestamp &t_b, const Eigen::MatrixXd &Phi_a_b) {
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

bool IIsolatedKalmanFilter::apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Factor){
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

// Eq. 20 in [1]
bool IIsolatedKalmanFilter::apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd Sigma_apos){
  Eigen::MatrixXd Lambda = Sigma_apos * Sigma_apri.inverse();
  return apply_correction_at_t(t, Lambda);
}

bool IIsolatedKalmanFilter::set_DICOHandler(std::shared_ptr<IDICOHandler> DICOHandler_ptr) {
  m_pHandler = DICOHandler_ptr;
  return true;
}

// Algorithm 7 in [1]
ProcessMeasResult_t IIsolatedKalmanFilter::delegate_measurement(const MeasData &m) {
  ProcessMeasResult_t res;

  if (m.obs_type == eObservationType::JOINT_OBSERVATION) {
    res.status = eMeasStatus::DISCARED;
    m_profiler.start();
    res = local_joint_measurement(m);
    res.exec_time = m_profiler.elapsedSec();
    res.t = m.t_m;
    res.meas_type = m.meas_type;
    res.obs_type = m.obs_type;
  } else {
    res = IKalmanFilter::delegate_measurement(m);
  }

  return res;
}

bool IIsolatedKalmanFilter::insert_measurement(const MeasData &m, const Timestamp &t) {
  return m_pHandler->insert_measurement(m, t);
}

// KF: Algorithm 8 in [1]
bool IIsolatedKalmanFilter::apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab,
                                              const Timestamp &t_a, const Timestamp &t_b) {
  if (IKalmanFilter::apply_propagation(Phi_II_ab, Q_II_ab, t_a, t_b)) {
    if (add_correction_at_t(t_a, t_b, Phi_II_ab)) {
      check_horizon();
      return true;
    }
    else {
      std::stringstream ss;
      ss << Phi_II_ab;
      Logger::ikf_logger()->warn("Could not set the correction factor Phi_II_ab=" + ss.str());
    }
  }
  return false;
}

// EKF: Algorithm 8 in [1]
bool IIsolatedKalmanFilter::apply_propagation(pBelief_t &bel_II_apri, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab,
                                              const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
  if (IKalmanFilter::apply_propagation(bel_II_apri, mean_II_b, Phi_II_ab, Q_II_ab, t_a, t_b)) {
    if (add_correction_at_t(t_a, t_b, Phi_II_ab)) {
      check_horizon();
      return true;
    }
    else {
      std::stringstream ss;
      ss << Phi_II_ab;
      Logger::ikf_logger()->warn("Could not set the correction factor Phi_II_ab=" + ss.str());
    }
  }
  return false;
}

bool ikf::IIsolatedKalmanFilter::apply_propagation(pBelief_t bel_II_b, const Eigen::MatrixXd &Phi_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
  if (IKalmanFilter::apply_propagation(bel_II_b, Phi_II_ab, t_a, t_b)) {
    if (add_correction_at_t(t_a, t_b, Phi_II_ab)) {
      check_horizon();
      return true;
    }
    else {
      std::stringstream ss;
      ss << Phi_II_ab;
      Logger::ikf_logger()->warn("Could not set the correction factor Phi_II_ab=" + ss.str());
    }
  }
  return false;
}

// KF:  Algorithm 4 in [1]
bool IIsolatedKalmanFilter::apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                                      const Eigen::VectorXd &z, const Timestamp &t, const KalmanFilter::CorrectionCfg_t &cfg) {
  pBelief_t bel_apri;
  if (get_belief_at_t(t, bel_apri)) {
    Eigen::VectorXd r = z - H_II * bel_apri->mean();

    return apply_private_observation(bel_apri, H_II, R, r, cfg);
  }
  return false;
}

// EKF:  Algorithm 4 in [1]
bool IIsolatedKalmanFilter::apply_private_observation(pBelief_t &bel_II_apri, const Eigen::MatrixXd &H_II,
                                                      const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
                                                      const KalmanFilter::CorrectionCfg_t &cfg) {
  auto t = bel_II_apri->timestamp();
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(bel_II_apri->Sigma()),
                      "Apri covariance is not PSD at t=" + t.str());

  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_II, R, r, bel_II_apri->Sigma(), cfg);

  if (!res.rejected) {
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(res.Sigma_apos), "Apos covariance is not PSD at t=" + t.str());

    // correction strategy: IMPORTANT: before setting the belief implace!
    if (apply_correction_at_t(t, res.U)) {

      bel_II_apri->correct(res.delta_mean, res.Sigma_apos);
      return true;
    } else {
      return false;
    }
  }
  return false;
}

bool IIsolatedKalmanFilter::apply_observation(const std::map<size_t, Eigen::MatrixXd> &dict_H, const Eigen::MatrixXd &R,
                                              const Eigen::VectorXd &r, const Timestamp &t,
                                              const KalmanFilter::CorrectionCfg_t &cfg) {
  return m_pHandler->apply_observation(dict_H, R, r, t, cfg);
}

bool IIsolatedKalmanFilter::apply_observation(const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t,
                                              H_joint_dx const &H, const std::vector<size_t> &IDs,
                                              const KalmanFilter::CorrectionCfg_t &cfg) {
  return m_pHandler->apply_observation(R, z, t, H, IDs, cfg);
}

}  // namespace ikf
