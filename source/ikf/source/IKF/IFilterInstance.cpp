/******************************************************************************
* FILENAME:     IFilterInstance.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     03.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/IKF/IFilterInstance.hpp>
#include <ikf/Sensor/Estimator/KalmanFilter.hpp>
namespace ikf {

IFilterInstance::IFilterInstance(const double horizon_sec_, const bool handle_delayed_meas) : HistBelief(horizon_sec_), HistMeas(horizon_sec_), max_time_horizon_sec(horizon_sec_), m_handle_delayed_meas(handle_delayed_meas) {

}

bool IFilterInstance::handle_delayed_meas() const {return m_handle_delayed_meas; }

void IFilterInstance::handle_delayed_meas(const bool val) {
  m_handle_delayed_meas = val;
  if (!m_handle_delayed_meas) {
    HistMeas.clear();
  }
}

void IFilterInstance::initialize(ptr_belief bel_init, const Timestamp &t) {
  reset();
  HistBelief.insert(bel_init, t);
}

ProcessMeasResult_t IFilterInstance::process_measurement(const MeasData &m) {
  // propagation and private -> to filter instance
  // joint -> use ptr_CIH and two filter instances: The CIH must provide others belief and ccf

  ProcessMeasResult_t res;
  res.rejected = true;
  switch(m.obs_type) {
    case eObservationType::PROPAGATION:
      {
        res = progapation_measurement(m);
        break;
      }
    case eObservationType::LOCAL_PRIVATE:
      {
        res = local_private_measurement(m);
        break;
      }
    case eObservationType::LOCAL_JOINT:
    case eObservationType::INTER_AGENT_JOINT:
    case eObservationType::UNKNOWN:
    default:
      res.rejected = true;
      break;
  }

  if (m_handle_delayed_meas) {
    if (!res.rejected && HistMeas.exist_after_t(m.t_m)) {
      redo_updates_after_t(m.t_m);
    }
    HistMeas.insert(m, m.t_m);
  }
  return res;

}

bool IFilterInstance::redo_updates_after_t(const Timestamp &t) {
  remove_beliefs_after_t(t);
  Timestamp t_after, t_last;
  if (HistMeas.get_after_t(t, t_after) &&  HistMeas.get_latest_t(t_last)) {
    HistMeas.foreach_between_t1_t2(t_after, t_last, [this](MeasData const&m){ this->process_measurement(m); });
    return true;
  }
  return false;
}

Timestamp IFilterInstance::current_t() const {
  Timestamp t;
  HistBelief.get_latest_t(t);
  return t;
}

bool IFilterInstance::exist_belief_at_t(const Timestamp &t) const {
  return HistBelief.exist_at_t(t);
}

ptr_belief IFilterInstance::get_belief_at_t(const Timestamp &t) const {
  ptr_belief bel;
  if (!HistBelief.get_at_t(t, bel)) {
    std::cout << "Iikf::get_belief_at_t: could not find belief at t=" << t << std::endl;
  }
  return bel;
}

bool IFilterInstance::get_belief_at_t(const Timestamp &t, ptr_belief &bel) {
  return HistBelief.get_at_t(t, bel);
}

void IFilterInstance::set_belief_at_t(const ptr_belief &bel, const Timestamp &t){
  HistBelief.insert(bel, t);
}

bool IFilterInstance::correct_belief_at_t(const Eigen::VectorXd &mean_corr, const Eigen::MatrixXd &Sigma_apos, const Timestamp &t){
  ptr_belief bel;
  bool res = get_belief_at_t(t, bel);
  if (res) {
    bel->correct(mean_corr, Sigma_apos);
    //set_belief_at_t(bel, t);
  }
  return res;
}

bool IFilterInstance::get_belief_before_t(const Timestamp &t, ptr_belief &bel, Timestamp &t_before)
{
  TStampedData<ptr_belief> tData;
  bool res = HistBelief.get_before_t(t, tData);
  bel = tData.data;
  t_before = tData.stamp;
  return res;
}

Eigen::VectorXd IFilterInstance::get_mean_at_t(const Timestamp &t) const {
  ptr_belief bel = get_belief_at_t(t);
  return bel->mean();
}

Eigen::MatrixXd IFilterInstance::get_Sigma_at_t(const Timestamp &t) const {
  ptr_belief bel = get_belief_at_t(t);
  return bel->Sigma();
}

void IFilterInstance::reset() {
  HistBelief.clear();
}

void IFilterInstance::remove_beliefs_after_t(const Timestamp &t) {
  HistBelief.remove_after_t(t);
}

void IFilterInstance::set_horizon(const double t_hor) {
  max_time_horizon_sec = t_hor;
  HistBelief.set_horizon(t_hor);
}

void IFilterInstance::check_horizon() {
  HistBelief.check_horizon();
}

bool IFilterInstance::apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab,
                                        const Timestamp &t_a, const Timestamp &t_b)  {
  ptr_belief bel_II_apri;
  if (get_belief_at_t(t_a, bel_II_apri)) {
    if(KalmanFilter::check_dim(bel_II_apri->mean(), Phi_II_ab)) {
      Eigen::VectorXd mean_II_b = Phi_II_ab * bel_II_apri->mean();
      return apply_propagation(bel_II_apri, mean_II_b, Phi_II_ab, Q_II_ab, t_a, t_b);
    }
  }
  else {
      std::cout << "No belief at t_a=" + t_a.str() + "! Did you forgot to initialize the filter?" << std::endl;
  }
  return false;
}

bool IFilterInstance::apply_propagation(ptr_belief &bel_II_apri, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab,
                                        const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
  if (KalmanFilter::check_dim(bel_II_apri->Sigma(),  Phi_II_ab, Q_II_ab)) {
    Eigen::MatrixXd Sigma_II_b = KalmanFilter::covariance_propagation(bel_II_apri->Sigma(),
                                                                      Phi_II_ab, Q_II_ab);
    // This is where the magic happens!
    ptr_belief bel_b = bel_II_apri->clone(); //clone the state definiton!
    if (KalmanFilter::check_dim(mean_II_b, Sigma_II_b) && bel_b->set(mean_II_b, Sigma_II_b)) {
      set_belief_at_t(bel_b, t_b);
      return true;
    }
  }
  else {
    std::cout << "Could not set the propagated belief from t_a=" << t_a << " to t_b=" << t_b << "! Maybe dimension missmatch?" << std::endl;
    std::cout << "Phi_II_ab=" << Phi_II_ab << "\n";
    std::cout << "Q_II_ab=" << Q_II_ab << "\n";
    std::cout << "Sigma_II_a=" << bel_II_apri->Sigma() << "\n";
    std::cout << "mean_II_a=" << bel_II_apri->mean() << std::endl;
  }
  return false;
}

//bool IFilterInstance::apply_propagation(const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
//  ptr_belief bel_a;
//  if (get_belief_at_t(t_a, bel_a)) {
//    return apply_propagation(bel_a, mean_II_b, Phi_II_ab, Q_II_ab, t_a, t_b);
//  }
//  else {
//    std::cout << "No belief at t_a=" + t_a.str() + "! Did you forgot to initialize the filter?" << std::endl;
//  }
//  return false;
//}

bool IFilterInstance::apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                                const Eigen::VectorXd &z, const Timestamp &t){
  ptr_belief bel_apri;
  if (get_belief_at_t(t, bel_apri)) {
    Eigen::VectorXd r = z - H_II * bel_apri->mean();
    return apply_private_observation(bel_apri, H_II, R, r, t);
  }
  return false;
}

bool IFilterInstance::apply_private_observation(ptr_belief &bel_II_apri, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                                const Eigen::VectorXd &r, const Timestamp &t) {
  KalmanFilter::CorrectionCfg_t cfg;
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_II, R, r, bel_II_apri->Sigma(), cfg);

  if (!res.rejected) {
    // inplace correction, no need to write belief in HistBelief
    bel_II_apri->correct(res.delta_mean, res.Sigma_apos);
  }
  return res.rejected;
}



} // ns mmsf
