/******************************************************************************
* FILENAME:     IKalmanFilter.cpp
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
******************************************************************************/
#include <ikf/Estimator/IKalmanFilter.hpp>
#include <ikf/Estimator/KalmanFilter.hpp>
namespace ikf {

IKalmanFilter::IKalmanFilter(const double horizon_sec_, const bool handle_delayed_meas) : HistBelief(horizon_sec_), HistMeas(horizon_sec_), HistMeasPropagation(horizon_sec_), max_time_horizon_sec(horizon_sec_), m_handle_delayed_meas(handle_delayed_meas) {

}

IKalmanFilter::IKalmanFilter(ptr_belief bel_0, const double horizon_sec_, const bool handle_delayed_meas) : HistBelief(horizon_sec_), HistMeas(horizon_sec_), HistMeasPropagation(horizon_sec_), max_time_horizon_sec(horizon_sec_), m_handle_delayed_meas(handle_delayed_meas)
{
  HistBelief.insert(bel_0, bel_0->timestamp());
}

bool IKalmanFilter::handle_delayed_meas() const {return m_handle_delayed_meas; }

void IKalmanFilter::handle_delayed_meas(const bool val) {
  m_handle_delayed_meas = val;
  if (!m_handle_delayed_meas) {
    HistMeas.clear();
  }
}

void IKalmanFilter::initialize(ptr_belief bel_init) {
  initialize(bel_init, bel_init->timestamp());
}

void IKalmanFilter::initialize(ptr_belief bel_init, const Timestamp &t) {
  reset();
  HistBelief.insert(bel_init, t);
}

ProcessMeasResult_t IKalmanFilter::process_measurement(const MeasData &m) {
  // propagation and private -> to filter instance
  // joint -> use ptr_CIH and two filter instances: The CIH must provide others belief and ccf

  auto res = IKalmanFilter::reprocess_measurement(m);

  if (m_handle_delayed_meas) {
    if (!res.rejected && HistMeas.exist_after_t(m.t_m)) {
      redo_updates_after_t(m.t_m);
    }
    HistMeas.insert(m, m.t_m);
  }

  // needed for inter-properation interpolation
  if (m.obs_type == eObservationType::PROPAGATION) {
    HistMeasPropagation.insert(m, m.t_m);
  }

  return res;
}

bool IKalmanFilter::redo_updates_after_t(const Timestamp &t) {
  remove_beliefs_after_t(t);
  Timestamp t_after, t_last;
  if (HistMeas.get_after_t(t, t_after) &&  HistMeas.get_latest_t(t_last)) {
    if (t_after == t_last) {
      MeasData m;
      HistMeas.get_at_t(t_after, m);
      this->reprocess_measurement(m);
    }
    else {
      HistMeas.foreach_between_t1_t2(t_after, t_last, [this](MeasData const&m){ this->reprocess_measurement(m); });
    }
    return true;
  }
  return false;
}

Timestamp IKalmanFilter::current_t() const {
  Timestamp t;
  HistBelief.get_latest_t(t);
  return t;
}

ptr_belief IKalmanFilter::current_belief() const {
  ptr_belief bel;
  if (HistBelief.get_latest(bel)) {
    return bel;
  }
  else {
    return ptr_belief(nullptr);
  }
}

bool IKalmanFilter::exist_belief_at_t(const Timestamp &t) const {
  return HistBelief.exist_at_t(t);
}

ptr_belief IKalmanFilter::get_belief_at_t(const Timestamp &t) const {
  ptr_belief bel;
  if (!HistBelief.get_at_t(t, bel)) {
    std::cout << "IKalmanFilter::get_belief_at_t: could not find belief at t=" << t << std::endl;
  }
  return bel;
}

bool IKalmanFilter::get_belief_at_t(const Timestamp &t, ptr_belief &bel) {
  if (!exist_belief_at_t(t)) {
      // TODO: propagate or interpolate between beliefs!
    ptr_belief bel_before;
    TStampedData<ptr_belief> stamped_data;
    if(HistBelief.get_before_t(t, stamped_data)){

      // TODO: simplification for now!
      bel = stamped_data.data->clone();
      bel->set_timestamp(t);
      HistBelief.insert(bel, t);
      return true;

//      if (propagate_from_to(stamped_data.stamp, t)) {
//        return HistBelief.get_at_t(t, bel);
//      } else {
//        std::cout << "Failed to propagate from t_a=" << stamped_data.stamp << " to t_b=" << t.str() << std::endl;
//        return false;
//      }
    }
    else {
      std::cout << "No belief found before  t=" + t.str() << std::endl;
      return false;
    }
  }
  else {
    return HistBelief.get_at_t(t, bel);
  }
}

void IKalmanFilter::set_belief_at_t(const ptr_belief &bel, const Timestamp &t){
  HistBelief.insert(bel, t);
}

bool IKalmanFilter::correct_belief_at_t(const Eigen::VectorXd &mean_corr, const Eigen::MatrixXd &Sigma_apos, const Timestamp &t){
  ptr_belief bel;
  bool res = get_belief_at_t(t, bel);
  if (res) {
    bel->correct(mean_corr, Sigma_apos);
    //set_belief_at_t(bel, t);
  }
  return res;
}

bool IKalmanFilter::get_belief_before_t(const Timestamp &t, ptr_belief &bel, Timestamp &t_before)
{
  TStampedData<ptr_belief> tData;
  bool res = HistBelief.get_before_t(t, tData);
  bel = tData.data;
  t_before = tData.stamp;
  return res;
}

Eigen::VectorXd IKalmanFilter::get_mean_at_t(const Timestamp &t) const {
  ptr_belief bel = get_belief_at_t(t);
  return bel->mean();
}

Eigen::MatrixXd IKalmanFilter::get_Sigma_at_t(const Timestamp &t) const {
  ptr_belief bel = get_belief_at_t(t);
  return bel->Sigma();
}

void IKalmanFilter::reset() {
  HistBelief.clear();
}

void IKalmanFilter::remove_beliefs_after_t(const Timestamp &t) {
  HistBelief.remove_after_t(t);
}

void IKalmanFilter::set_horizon(const double t_hor) {
  max_time_horizon_sec = t_hor;
  HistBelief.set_horizon(t_hor);
}

void IKalmanFilter::check_horizon() {
  HistBelief.check_horizon();
}

void IKalmanFilter::print_HistMeas(size_t max, bool reverse) {
  size_t cnt = 0;
  auto lambda = [&cnt, max](MeasData const& i){
    if(cnt < max) {
      std::cout << "* " << i << std::endl;
    }
    cnt++;
  };
  if (!reverse) {
    HistMeas.foreach(lambda);
  } else {
    HistMeas.foreach_reverse(lambda);
  }

}

void IKalmanFilter::print_HistBelief(size_t max, bool reverse) {
  size_t cnt = 0;
  auto lambda = [&cnt, max](ptr_belief const& i){
    if(cnt < max) {
      std::cout << (*i.get()) << std::endl;
    }
    cnt++;
  };
  if (!reverse) {
    HistBelief.foreach(lambda);
  } else {
    HistBelief.foreach_reverse(lambda);
  }
}

ProcessMeasResult_t IKalmanFilter::reprocess_measurement(const MeasData &m) {
  ProcessMeasResult_t res;
  res.rejected = true;
  switch(m.obs_type) {
    case eObservationType::PROPAGATION:
      {
        res = progapation_measurement(m);
        break;
      }
    case eObservationType::PRIVATE_OBSERVATION:
      {
        res = local_private_measurement(m);
        break;
      }
    case eObservationType::JOINT_OBSERVATION:
    case eObservationType::UNKNOWN:
    default:
      res.rejected = true;
      break;
  }
  return res;
}

bool IKalmanFilter::propagate_from_to(const Timestamp &t_a, const Timestamp &t_b) {

  TStampedData<MeasData> m_a, m_c;
  m_a.stamp = t_a;
  if(!HistMeasPropagation.get_at_t(t_a, m_a.data)) {
    if(!HistMeasPropagation.get_before_t(t_a, m_a)) {
      return false;
    }
  }
  m_c.stamp = t_b;
  if(!HistMeasPropagation.get_at_t(t_b, m_c.data)) {
    if(!HistMeasPropagation.get_after_t(t_b, m_c)) {
      return false;
    }
  }

  double const dt = m_c.stamp.to_sec() - m_a.stamp.to_sec();
  double const d_ab = t_b.to_sec() - m_a.stamp.to_sec();
  double const ratio = d_ab /dt;
  std::cout << "IKalmanFilter: propagate from t_a=" << m_a.stamp << " to t_b=" << t_b << std::endl;
  std::cout << "\t bounded between t_a=" << m_a.stamp << " and  t_c=" << m_c.stamp << ", ratio=" << ratio <<std::endl;


  MeasData pseudo_meas_b = m_a.data;
  pseudo_meas_b.t_m = t_b;
  pseudo_meas_b.t_p = t_b;
  pseudo_meas_b.z = m_a.data.z + (m_c.data.z - m_a.data.z)*ratio;
  pseudo_meas_b.R = m_a.data.R + (m_c.data.R - m_a.data.R)*ratio;


  process_measurement(pseudo_meas_b);
  return true;
}

bool IKalmanFilter::apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab,
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

bool IKalmanFilter::apply_propagation(ptr_belief &bel_II_apri, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab,
                                      const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
  if (KalmanFilter::check_dim(bel_II_apri->Sigma(),  Phi_II_ab, Q_II_ab)) {
    Eigen::MatrixXd Sigma_II_b = KalmanFilter::covariance_propagation(bel_II_apri->Sigma(),
                                                                      Phi_II_ab, Q_II_ab);
    // This is where the magic happens!
    ptr_belief bel_b = bel_II_apri->clone(); //clone the state definiton!

    if (KalmanFilter::check_dim(mean_II_b, Sigma_II_b) && bel_b->set(mean_II_b, Sigma_II_b)) {
      bel_b->set_timestamp(t_b);
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

//bool IKalmanFilter::apply_propagation(const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
//  ptr_belief bel_a;
//  if (get_belief_at_t(t_a, bel_a)) {
//    return apply_propagation(bel_a, mean_II_b, Phi_II_ab, Q_II_ab, t_a, t_b);
//  }
//  else {
//    std::cout << "No belief at t_a=" + t_a.str() + "! Did you forgot to initialize the filter?" << std::endl;
//  }
//  return false;
//}

bool IKalmanFilter::apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                                const Eigen::VectorXd &z, const Timestamp &t){
  ptr_belief bel_apri;
  if (get_belief_at_t(t, bel_apri)) {
    Eigen::VectorXd r = z - H_II * bel_apri->mean();
    return apply_private_observation(bel_apri, H_II, R, r);
  }
  return false;
}

bool IKalmanFilter::apply_private_observation(ptr_belief &bel_II_apri, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &r) {
  KalmanFilter::CorrectionCfg_t cfg;
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_II, R, r, bel_II_apri->Sigma(), cfg);

  if (!res.rejected) {
    // inplace correction, no need to write belief in HistBelief
    bel_II_apri->correct(res.delta_mean, res.Sigma_apos);
  }
  return !res.rejected;
}



} // ns mmsf
