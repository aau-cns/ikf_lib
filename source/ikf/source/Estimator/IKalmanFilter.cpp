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
#include <ikf/utils/eigen_utils.hpp>
#include <ikf/Logger/Logger.hpp>

namespace ikf {

IKalmanFilter::IKalmanFilter(const double horizon_sec_, const bool handle_delayed_meas) : HistBelief(horizon_sec_), HistMeas(horizon_sec_), HistMeasPropagation(horizon_sec_), max_time_horizon_sec(horizon_sec_), m_handle_delayed_meas(handle_delayed_meas) {

}

IKalmanFilter::IKalmanFilter(pBelief_t bel_0, const double horizon_sec_, const bool handle_delayed_meas) : HistBelief(horizon_sec_), HistMeas(horizon_sec_), HistMeasPropagation(horizon_sec_), max_time_horizon_sec(horizon_sec_), m_handle_delayed_meas(handle_delayed_meas)
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

bool IKalmanFilter::enabled() const { return m_enabled; }

void IKalmanFilter::enabled(const bool val) { m_enabled = val; }

void IKalmanFilter::initialize(pBelief_t bel_init) {
  initialize(bel_init, bel_init->timestamp());
}

void IKalmanFilter::initialize(pBelief_t bel_init, const Timestamp &t) {
  reset();
  HistBelief.insert(bel_init, t);
}

ProcessMeasResult_vec_t IKalmanFilter::process_measurement(const MeasData &m) {
  // propagation and private -> to filter instance
  // joint -> use ptr_CIH and two filter instances: The CIH must provide others belief and ccf

  auto res = IKalmanFilter::delegate_measurement(m);
  ProcessMeasResult_vec_t vec({res});
  if (m_handle_delayed_meas) {
    if (res.status == eMeasStatus::OUTOFORDER && HistMeas.exist_after_t(m.t_m)) {
      auto vec_after = redo_updates_after_t(m.t_m);
      vec.insert(vec.end(), vec_after.begin(), vec_after.end());
    }
    if (res.status != eMeasStatus::DISCARED) {
      insert_measurement(m, m.t_m);
      HistMeas.check_horizon();
    }
  }
  return vec;
}

ProcessMeasResult_vec_t IKalmanFilter::redo_updates_after_t(const Timestamp &t) {
  remove_beliefs_after_t(t);
  Timestamp t_after, t_last;
  ProcessMeasResult_vec_t vec;
  if (HistMeas.get_after_t(t, t_after) &&  HistMeas.get_latest_t(t_last)) {
    Logger::ikf_logger()->trace("IKalmanFilter::redo_updates_after_t() t_after=" + t_after.str() + ", t_last=" + t_last.str());
    if (t_after == t_last) {
      MeasData m;
      HistMeas.get_at_t(t_after, m);
      vec.push_back(this->delegate_measurement(m));
    }
    else {
      HistMeas.foreach_between_t1_t2(t_after, t_last,
                                     [this, &vec](MeasData const &m) { vec.push_back(this->delegate_measurement(m)); });
    }
    return vec;
  }
  return vec;
}

Timestamp IKalmanFilter::current_t() const {
  Timestamp t;
  if (HistBelief.size()) {
    HistBelief.get_latest_t(t);
  }
  return t;
}

pBelief_t IKalmanFilter::current_belief() const {
  pBelief_t bel;
  if (HistBelief.size() && HistBelief.get_latest(bel)) {
    return bel;
  } else {
    return pBelief_t(nullptr);
  }
}

bool IKalmanFilter::exist_belief_at_t(const Timestamp &t) const {
  return HistBelief.exist_at_t(t);
}

pBelief_t ikf::IKalmanFilter::get_belief_at_t(const Timestamp &t) const {
  pBelief_t bel;
  if (!HistBelief.get_at_t(t, bel)) {
    Logger::ikf_logger()->info("IKalmanFilter::get_belief_at_t: could not find belief at t=" + t.str());
  }
  return bel;
}


pBelief_t IKalmanFilter::get_belief_at_t(const Timestamp &t, const ikf::eGetBeliefStrategy type) {
  pBelief_t bel;
  if (!get_belief_at_t(t, bel, type)) {
    Logger::ikf_logger()->info("IKalmanFilter::get_belief_at_t: could not find belief at t=" + t.str());
  }
  return bel;
}

bool IKalmanFilter::get_belief_at_t(const Timestamp &t, pBelief_t &bel, const ikf::eGetBeliefStrategy type) {
  if (!exist_belief_at_t(t)) {
    switch(type) {
      case eGetBeliefStrategy::EXACT:
      {
        return HistBelief.get_at_t(t, bel);
      }
      case eGetBeliefStrategy::CLOSEST:
      {
        TStampedData<pBelief_t> stamped_bel_prev, stamped_bel_after;
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
        } else {
          Logger::ikf_logger()->debug(
            "IKalmanFilter::get_belief_at_t: NO BOUNDING measurements for CLOSEST found! at t=" + t.str());
          //  no bounds
          return false;
        }
        break;
      }
      case eGetBeliefStrategy::LINEAR_INTERPOL_BELIEF:
      {
        TStampedData<pBelief_t> stamped_bel_prev, stamped_bel_after;
        if(HistBelief.get_before_t(t, stamped_bel_prev) && HistBelief.get_after_t(t, stamped_bel_after))
        {

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
        if (HistMeasPropagation.size() > 1) {
          TStampedData<MeasData> stamped_meas_prev, stamped_meas_after;
          if(HistMeasPropagation.get_before_t(t, stamped_meas_prev) && HistMeasPropagation.get_after_t(t, stamped_meas_after)) {
            // bounded between two measurements

            if (!HistBelief.exist_at_t(stamped_meas_prev.stamp)) {
              Logger::ikf_logger()->debug("IKalmanFilter::get_belief_at_t: No blief exist at t= "
                                          + stamped_meas_prev.stamp.str() + " in LINEAR_INTERPOL_MEAS");

              Logger::ikf_logger()->debug(
                "IKalmanFilter::get_belief_at_t: NO BOUNDING measurements for LINEAR_INTERPOL_MEAS found! at t="
                + t.str());
              return false;
            }
            MeasData pseudo_meas_b = MeasData::lin_interpolate(stamped_meas_prev.data, stamped_meas_after.data, t);

            // ACCESS TO MODEL
            ikf::ProcessMeasResult_t res = progapation_measurement(pseudo_meas_b);

            HistMeasPropagation.insert(pseudo_meas_b, t);

            // HOOK for child classes (IsolatedKalmanFilter)
            if(!RTV_EXPECT_TRUE_MSG(insert_measurement(pseudo_meas_b, t), "IKalmanFilter::get_belief_at_t(): Pseudo measurement cannot be added at t=" + t.str()))
            {
              return false;
            }

            if (res.status == eMeasStatus::REJECTED) {
              Logger::ikf_logger()->debug(
                "IKalmanFilter::get_belief_at_t: pseudo measurement for LINEAR_INTERPOL_MEAS was REJECTED! at t="
                + t.str());
            }

            // if not rejected, it will insert a new element into HistBeliefs
            return (res.status == eMeasStatus::PROCESSED) && HistBelief.get_at_t(t, bel);
          } else {
            Logger::ikf_logger()->debug(
              "IKalmanFilter::get_belief_at_t: NO BOUNDING measurements for LINEAR_INTERPOL_MEAS found! at t="
              + t.str());
            return false;
          }
        } else {
          // no proprioceptive measurements available!
          Logger::ikf_logger()->debug(
            "IKalmanFilter::get_belief_at_t: NO MEASUREMENTS for LINEAR_INTERPOL_MEAS found! at t=" + t.str());
          return false;
        }
        break;
      }
      case eGetBeliefStrategy::PREDICT_BELIEF:
      {
        // if true, it will insert a new element into HistBeliefs
        bool res =  predict_to(t);
        return res && HistBelief.get_at_t(t, bel);
      }

      case eGetBeliefStrategy::AUTO: {
        bool res = false;
        if (HistMeasPropagation.size() > 1) {
          res = get_belief_at_t(t, bel, eGetBeliefStrategy::LINEAR_INTERPOL_MEAS);
        }
        if (!res) {
          res = get_belief_at_t(t, bel, eGetBeliefStrategy::PREDICT_BELIEF);
        }
        return res;
      }
      default:
      {
        return false;
      }
    }
    return false;
  } else {
    // exact blief exists!s
    return HistBelief.get_at_t(t, bel);
  }
}

void IKalmanFilter::set_belief_at_t(const pBelief_t &bel, const Timestamp &t){
  HistBelief.insert(bel, t);
}

bool IKalmanFilter::correct_belief_at_t(const Eigen::VectorXd &mean_corr, const Eigen::MatrixXd &Sigma_apos, const Timestamp &t){
  pBelief_t bel;
  bool res = get_belief_at_t(t, bel);
  if (res) {
    bel->correct(mean_corr, Sigma_apos);
    //set_belief_at_t(bel, t);
  }
  return res;
}

bool IKalmanFilter::get_belief_before_t(const Timestamp &t, pBelief_t &bel, Timestamp &t_before)
{
  TStampedData<pBelief_t> tData;
  bool res = HistBelief.get_before_t(t, tData);
  bel = tData.data;
  t_before = tData.stamp;
  return res;
}

Eigen::VectorXd IKalmanFilter::get_mean_at_t(const Timestamp &t) const {
  pBelief_t bel = get_belief_at_t(t);
  return bel->mean();
}

Eigen::MatrixXd IKalmanFilter::get_Sigma_at_t(const Timestamp &t) const {
  pBelief_t bel = get_belief_at_t(t);
  return bel->Sigma();
}

void IKalmanFilter::reset() {
  HistBelief.clear();
  // IMPORTANT: DO NOT CLEAR
  // HistMeas.clear();
  // HistMeasPropagation.clear();
}

bool IKalmanFilter::insert_measurement(const MeasData &m, const Timestamp &t) {
  if(m_handle_delayed_meas) {
    HistMeas.insert(m, t);
  }
  return m_handle_delayed_meas;
}

void IKalmanFilter::remove_beliefs_after_t(const Timestamp &t) {
  HistBelief.remove_after_t(t);
}

void IKalmanFilter::set_horizon(const double t_hor) {
  max_time_horizon_sec = t_hor;
  HistBelief.set_horizon(t_hor);
  HistMeas.set_horizon(t_hor);
  HistMeasPropagation.set_horizon(t_hor);
}

void IKalmanFilter::check_horizon() {
  HistBelief.check_horizon();
  HistMeas.check_horizon();
  HistMeasPropagation.check_horizon();
}

void IKalmanFilter::print_HistMeas(size_t max, bool reverse) {
  size_t cnt = 0;
  auto lambda = [&cnt, max](MeasData const& i){
    if(cnt < max) {
      std::stringstream ss;
      ss << "* " << i;
      Logger::ikf_logger()->info(ss.str());
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
  auto lambda = [&cnt, max](pBelief_t const& i){
    if(cnt < max) {
      std::stringstream ss;
      ss << (*i.get()) ;
      Logger::ikf_logger()->info(ss.str());
    }
    cnt++;
  };
  if (!reverse) {
    HistBelief.foreach(lambda);
  } else {
    HistBelief.foreach_reverse(lambda);
  }
}

bool ikf::IKalmanFilter::get_prop_meas_at_t(const Timestamp &t, MeasData &m) {
  if(HistMeasPropagation.exist_at_t(t)) {
    HistMeasPropagation.get_at_t(t, m);
    return true;
  }
  return false;
}

ProcessMeasResult_t IKalmanFilter::delegate_measurement(const MeasData &m) {
  ProcessMeasResult_t res;
  res.status = eMeasStatus::DISCARED;
  switch (m.obs_type) {
  case eObservationType::PROPAGATION: {
    res = progapation_measurement(m);

    // needed for inter-properation interpolation (replace in case of re-do updates
    HistMeasPropagation.insert(m, m.t_m);
    break;
  }
  case eObservationType::PRIVATE_OBSERVATION: {
    res = local_private_measurement(m);
    break;
  }
    case eObservationType::JOINT_OBSERVATION:
    case eObservationType::UNKNOWN:
    default:
    break;
    }

    res.t = m.t_m;
    res.observation_type = m.meas_type;
    return res;
}

bool IKalmanFilter::apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab,
                                      const Timestamp &t_a, const Timestamp &t_b)  {
  pBelief_t bel_II_apri;
  if (get_belief_at_t(t_a, bel_II_apri)) {
    if(KalmanFilter::check_dim(bel_II_apri->mean(), Phi_II_ab)) {
      Eigen::VectorXd mean_II_b = Phi_II_ab * bel_II_apri->mean();
      return apply_propagation(bel_II_apri, mean_II_b, Phi_II_ab, Q_II_ab, t_a, t_b);
    }
  }
  else {
    Logger::ikf_logger()->warn("No belief at t_a=" + t_a.str() + "! Did you forgot to initialize the filter?");
  }
  return false;
}

bool IKalmanFilter::apply_propagation(pBelief_t &bel_II_a, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab,
                                      const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
  if (KalmanFilter::check_dim(bel_II_a->Sigma(),  Phi_II_ab, Q_II_ab)) {
    Eigen::MatrixXd Sigma_II_b = KalmanFilter::covariance_propagation(bel_II_a->Sigma(),
                                                                      Phi_II_ab, Q_II_ab);

    //RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_II_b), "[ERROR] Not PSD: Covariance propagted to t_b=" + t_b.str());

    // This is where the magic happens!
    pBelief_t bel_b = bel_II_a->clone(); //clone the state definiton!

    if (KalmanFilter::check_dim(mean_II_b, Sigma_II_b) && bel_b->set(mean_II_b, Sigma_II_b)) {
      bel_b->set_timestamp(t_b);
      set_belief_at_t(bel_b, t_b);
      return true;
    }
  }
  else {
    std::stringstream ss;
    ss << "Could not set the propagated belief from t_a=" << t_a << " to t_b=" << t_b << "! Maybe dimension missmatch?" << std::endl;
    ss << "Phi_II_ab=" << Phi_II_ab << "\n";
    ss << "Q_II_ab=" << Q_II_ab << "\n";
    ss << "Sigma_II_a=" << bel_II_a->Sigma() << "\n";
    ss << "mean_II_a=" << bel_II_a->mean();
    Logger::ikf_logger()->error(ss.str());
  }
  return false;
}

bool ikf::IKalmanFilter::apply_propagation(ikf::pBelief_t bel_II_b, const Eigen::MatrixXd &Phi_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
  if (KalmanFilter::check_dim(bel_II_b->Sigma(),  Phi_II_ab)) {
    //RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(bel_II_b->Sigma()), "[ERROR] Not PSD: Covariance propagted to t_b=" + t_b.str());

    bel_II_b->set_timestamp(t_b);
    set_belief_at_t(bel_II_b, t_b);
    return true;
  }
  else {
    std::stringstream ss;
    ss << "Could not set the propagated belief from t_a=" << t_a << " to t_b=" << t_b << "! Maybe dimension missmatch?" << std::endl;
    ss << "Phi_II_ab=" << Phi_II_ab << "\n";
    ss << "Sigma_II_b=" << bel_II_b->Sigma() << "\n";
    ss << "mean_II_b=" << bel_II_b->mean() << std::endl;
    Logger::ikf_logger()->error(ss.str());
  }
  return false;
}

//bool IKalmanFilter::apply_propagation(const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
//  pBelief_t bel_a;
//  if (get_belief_at_t(t_a, bel_a)) {
//    return apply_propagation(bel_a, mean_II_b, Phi_II_ab, Q_II_ab, t_a, t_b);
//  }
//  else {
//    std::cout << "No belief at t_a=" + t_a.str() + "! Did you forgot to initialize the filter?" << std::endl;
//  }
//  return false;
//}

bool IKalmanFilter::apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                                const Eigen::VectorXd &z, const Timestamp &t, const ikf::KalmanFilter::CorrectionCfg_t &cfg){
  pBelief_t bel_apri;
  if (get_belief_at_t(t, bel_apri)) {
    Eigen::VectorXd r = z - H_II * bel_apri->mean();
    return apply_private_observation(bel_apri, H_II, R, r, cfg);
  }
  return false;
}

bool IKalmanFilter::apply_private_observation(pBelief_t &bel_II_apri, const Eigen::MatrixXd &H_II,
                                              const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
                                              const ikf::KalmanFilter::CorrectionCfg_t &cfg) {
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_II, R, r, bel_II_apri->Sigma(), cfg);

  if (!res.rejected) {
    // inplace correction, no need to write belief in HistBelief
    bel_II_apri->correct(res.delta_mean, res.Sigma_apos);
  }
  return !res.rejected;
}

eGetBeliefStrategy str2eGetBeliefStrategy(const std::string &str) {
  if (str == "EXACT") {
    return eGetBeliefStrategy::EXACT;
  } else if (str == "CLOSEST") {
    return eGetBeliefStrategy::CLOSEST;
  } else if (str == "LINEAR_INTERPOL_BELIEF") {
    return eGetBeliefStrategy::LINEAR_INTERPOL_BELIEF;
  } else if (str == "LINEAR_INTERPOL_MEAS") {
    return eGetBeliefStrategy::LINEAR_INTERPOL_MEAS;
  } else if (str == "PREDICT_BELIEF") {
    return eGetBeliefStrategy::PREDICT_BELIEF;
  }
  return eGetBeliefStrategy::EXACT;
}

std::string to_string(const eGetBeliefStrategy e) {
  switch (e) {
  case eGetBeliefStrategy::EXACT:
    return "EXACT";
  case eGetBeliefStrategy::CLOSEST:
    return "CLOSEST";
  case eGetBeliefStrategy::LINEAR_INTERPOL_BELIEF:
    return "LINEAR_INTERPOL_BELIEF";
  case eGetBeliefStrategy::LINEAR_INTERPOL_MEAS:
    return "LINEAR_INTERPOL_MEAS";
  case eGetBeliefStrategy::PREDICT_BELIEF:
    return "PREDICT_BELIEF";
  default:
    break;
  }
  return "";
}

}  // namespace ikf
