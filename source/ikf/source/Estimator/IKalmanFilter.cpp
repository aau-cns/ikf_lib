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

ptr_belief ikf::IKalmanFilter::get_belief_at_t(const Timestamp &t) const {
  ptr_belief bel;
  if (!HistBelief.get_at_t(t, bel)) {
    std::cout << "IKalmanFilter::get_belief_at_t: could not find belief at t=" << t << std::endl;
  }
  return bel;
}


ptr_belief IKalmanFilter::get_belief_at_t(const Timestamp &t, const ikf::eGetBeliefStrategy type) {
  ptr_belief bel;
  if (!get_belief_at_t(t, bel, type)) {
    std::cout << "IKalmanFilter::get_belief_at_t: could not find belief at t=" << t << std::endl;
  }
  return bel;
}

bool IKalmanFilter::get_belief_at_t(const Timestamp &t, ptr_belief &bel, const ikf::eGetBeliefStrategy type) {
  if (!exist_belief_at_t(t)) {
    switch(type) {
      case eGetBeliefStrategy::EXACT:
      {
        return HistBelief.get_at_t(t, bel);
      }
      case eGetBeliefStrategy::CLOSEST:
      {
        TStampedData<ptr_belief> stamped_bel_prev, stamped_bel_after;
        if(HistBelief.get_before_t(t, stamped_bel_prev)) {
          if (HistBelief.get_after_t(t, stamped_bel_after)) {
            // bounded between two beliefs

            std::int64_t dt_prev_ns = t.stamp_ns() - stamped_bel_prev.stamp.stamp_ns();
            std::int64_t dt_after_ns = stamped_bel_after.stamp.stamp_ns() - t.stamp_ns();
            if (dt_prev_ns <= dt_after_ns) {
              // choose lower bound
              bel = stamped_bel_prev.data;
            } else {
              // choose upper bound
              bel = stamped_bel_after.data;
            }
            return true;
          }
          else {
            // lower bound
            bel = stamped_bel_prev.data;
            return true;
          }
        }
        else if (HistBelief.get_after_t(t, stamped_bel_after)) {
          // upper bound
          bel = stamped_bel_after.data;
          return true;
        }
        else {
          //  no bounds
          return false;
        }
        break;
      }
      case eGetBeliefStrategy::LINEAR_INTERPOL_BELIEF:
      {
        TStampedData<ptr_belief> stamped_bel_prev, stamped_bel_after;
        if(HistBelief.get_before_t(t, stamped_bel_prev) && HistBelief.get_after_t(t, stamped_bel_after)) {

          // bounded between two beliefs
          double const i = (t.to_sec() - stamped_bel_prev.stamp.to_sec())/(stamped_bel_after.stamp.to_sec() - stamped_bel_prev.stamp.to_sec());
          bel = stamped_bel_prev.data->interpolate(stamped_bel_prev.data, stamped_bel_after.data, i);

          // INFO: this lead to a "state transition" and needs to trigger apply_propagation (as hook for other filter approaches to track cross-covariances)!
          // insert new element into HistBeliefs
          Eigen::MatrixXd Lambda = bel->Sigma() * stamped_bel_prev.data->Sigma().inverse();
          return apply_propagation(bel, Lambda, stamped_bel_prev.stamp, t);
        }
        else {
          //  no bounds
          return false;
        }
        break;
      }
      case eGetBeliefStrategy::LINEAR_INTERPOL_MEAS:
      {
        if(HistMeasPropagation.size() > 0) {
          TStampedData<MeasData> stamped_meas_prev, stamped_meas_after;
          if(HistMeasPropagation.get_before_t(t, stamped_meas_prev) && HistMeasPropagation.get_after_t(t, stamped_meas_after)) {
            // bounded between two measurements

            RTV_EXPECT_TRUE_THROW(HistBelief.exist_at_t(stamped_meas_prev.stamp), "No blief exist for get_belief()  + LINEAR_INTERPOL_MEAS!");
            MeasData pseudo_meas_b = MeasData::lin_interpolate(stamped_meas_prev.data, stamped_meas_after.data, t);

            ikf::ProcessMeasResult_t res = progapation_measurement(pseudo_meas_b);

            // if not rejected, it will insert a new element into HistBeliefs
            return !res.rejected && HistBelief.get_at_t(t, bel);
          }
          else {
            //  no bounds
            return false;
          }
        }
        else {
          // no proprioceptive measurements available!
          return false;
        }
      }
      case eGetBeliefStrategy::PREDICT_BELIEF:
      {
        // if true, it will insert a new element into HistBeliefs
        bool res =  predict_to(t);
        return res && HistBelief.get_at_t(t, bel);
      }
    }
  } else {
    // exact blief exists!s
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

        // needed for inter-properation interpolation (replace in case of re-do updates
        HistMeasPropagation.insert(m, m.t_m);
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
      break;
  }
  return res;
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

bool IKalmanFilter::apply_propagation(ptr_belief &bel_II_a, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab,
                                      const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
  if (KalmanFilter::check_dim(bel_II_a->Sigma(),  Phi_II_ab, Q_II_ab)) {
    Eigen::MatrixXd Sigma_II_b = KalmanFilter::covariance_propagation(bel_II_a->Sigma(),
                                                                      Phi_II_ab, Q_II_ab);
    // This is where the magic happens!
    ptr_belief bel_b = bel_II_a->clone(); //clone the state definiton!

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
    std::cout << "Sigma_II_a=" << bel_II_a->Sigma() << "\n";
    std::cout << "mean_II_a=" << bel_II_a->mean() << std::endl;
  }
  return false;
}

bool ikf::IKalmanFilter::apply_propagation(ikf::ptr_belief bel_II_b, const Eigen::MatrixXd &Phi_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
  if (KalmanFilter::check_dim(bel_II_b->Sigma(),  Phi_II_ab)) {
    bel_II_b->set_timestamp(t_b);
    set_belief_at_t(bel_II_b, t_b);
    return true;
  }
  else {
    std::cout << "Could not set the propagated belief from t_a=" << t_a << " to t_b=" << t_b << "! Maybe dimension missmatch?" << std::endl;
    std::cout << "Phi_II_ab=" << Phi_II_ab << "\n";
    std::cout << "Sigma_II_b=" << bel_II_b->Sigma() << "\n";
    std::cout << "mean_II_b=" << bel_II_b->mean() << std::endl;
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
