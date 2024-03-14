/******************************************************************************
 * FILENAME:     IDICOHandler.cpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     10.08.2023
 *
 *  Copyright (C) 2023
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#include <ikf/EstimatorHandler/IDICOHandler.hpp>

namespace ikf {

IDICOHandler::IDICOHandler(const double horizon_sec) : HistMeas(horizon_sec), m_horzion_sec(horizon_sec) {
  Logger::ikf_logger()->info("IDICOHandler will handle delayed measurements for all it's instances (centralized)!");
  Logger::ikf_logger()->info("IDICOHandler: m_horizon_sec=" + std::to_string(m_horzion_sec));
}

bool IDICOHandler::add(pIKF_t p_IKF) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
  if (!exists(p_IKF->ID())) {
    // either one is handling delayed measurements!
    p_IKF->handle_delayed_meas(false);
    p_IKF->set_horizon(m_horzion_sec);
    id_dict.emplace(p_IKF->ID(), p_IKF);
    return true;
  }
  return false;
}

bool IDICOHandler::remove(const size_t ID) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  auto elem = get(ID);
  if (elem) {
    id_dict.erase(elem->ID());
    return true;
  }
  return false;
}

bool IDICOHandler::exists(const size_t ID) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
  return (id_dict.find(ID) != id_dict.end());
}

std::vector<size_t> IDICOHandler::get_instance_ids() {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  std::vector<size_t> IDs;
  IDs.reserve(id_dict.size());
  for (auto const &elem : id_dict) {
    IDs.push_back(elem.first);
  }
  return IDs;
}

std::vector<std::string> IDICOHandler::get_instance_types() {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  std::vector<std::string> IDs;
  IDs.reserve(id_dict.size());
  for (auto const &elem : id_dict) {
    IDs.push_back(elem.second->m_type);
  }
  return IDs;
}

double ikf::IDICOHandler::horizon_sec() const { return m_horzion_sec; }

void IDICOHandler::set_horizon(const double t_hor) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  m_horzion_sec = t_hor;
  for (auto const &elem : id_dict) {
    elem.second->set_horizon(m_horzion_sec);
  }
  HistMeas.set_horizon(m_horzion_sec);
}

bool ikf::IDICOHandler::insert_measurement(const MeasData &m, const Timestamp &t) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  HistMeas.insert(m, t);
  return true;
}

void IDICOHandler::print_HistMeas(std::ostream &out, size_t max) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  size_t cnt = 0;
  out << "|\t t_p \t |\t t_m \t |\t ID \t |\t type \t|" << std::endl;
  HistMeas.foreach ([&cnt, max, &out](ikf::MeasData const &i) {
    if (cnt < max) {
      out << "|\t " << std::setprecision(4) << i.t_p.to_sec() << " \t |\t " << std::setprecision(4) << i.t_m.to_sec()
          << " \t |\t " << i.id_sensor << " \t |\t" << ikf::to_string(i.obs_type) << "\t |" << std::endl;
    }
    cnt++;
  });
}

void IDICOHandler::sort_measurements_from_t(const Timestamp &t) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  Timestamp t_latest;
  if (HistMeas.get_latest_t(t_latest)) {
    TMultiHistoryBuffer<MeasData> meas = HistMeas.get_between_t1_t2(t, t_latest);

    if (!meas.empty()) {
      HistMeas.remove_after_t(t);
      HistMeas.remove_at_t(t);

      // first insert all PROPAGATION sorted
      meas.foreach ([this](MeasData const &elem) {
        if (elem.obs_type == eObservationType::PROPAGATION) {
          HistMeas.insert(elem, elem.t_m);
        }
      });

      // second insert all PRIVATE sorted
      meas.foreach ([this](MeasData const &elem) {
        if (elem.obs_type == eObservationType::PRIVATE_OBSERVATION) {
          HistMeas.insert(elem, elem.t_m);
        }
      });

      // third insert all JOINT sorted
      meas.foreach ([this](MeasData const &elem) {
        if (elem.obs_type == eObservationType::JOINT_OBSERVATION) {
          HistMeas.insert(elem, elem.t_m);
        }
      });
    }
  }
}

ProcessMeasResult_vec_t IDICOHandler::process_measurement(const MeasData &m) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);
  if (!exists(m.id_sensor)) {
    return ProcessMeasResult_vec_t({ProcessMeasResult_t(eMeasStatus::DISCARED)});
  }

  Timestamp t_latest;
  if (HistMeas.get_latest_t(t_latest)) {
    if (t_latest.to_sec() - m.t_m.to_sec() >= m_horzion_sec) {
      ikf::Logger::ikf_logger()->warn("IDICOHandler::process_measurement(): measurement is too much delayed");
      return ProcessMeasResult_vec_t({ProcessMeasResult_t(eMeasStatus::DISCARED)});
    }
  }

  // if there are open request, use the oldest one, before processing the new measurmeent
  if (HistRedoUpdateRequest.size()) {
    Timestamp t_oldest;
    {
      std::scoped_lock lk(m_mtx_histRUR);
      HistRedoUpdateRequest.get_oldest_t(t_oldest);
      HistRedoUpdateRequest.clear();
    }
    redo_updates_after_t(t_oldest);
    ikf::Logger::ikf_logger()->info(
      "IDICOHandler::process_measurement(): requested redo update after t=" + t_oldest.str() + " processed!");
  }

  // concurrent order
  bool order_violated = is_order_violated(m);
  if (order_violated) {
    insert_measurement(m, m.t_m);
    sort_measurements_from_t(m.t_m);
    auto vec = redo_updates_from_t(m.t_m);
    HistMeas.check_horizon();
    return vec;
  } else {
    ProcessMeasResult_t res = delegate_measurement(m);
    ProcessMeasResult_vec_t vec({res});
    if (res.status == eMeasStatus::OUTOFORDER && HistMeas.exist_after_t(m.t_m)) {
      auto vec_after = redo_updates_after_t(m.t_m);
      vec.insert(vec.end(), vec_after.begin(), vec_after.end());
    }
    if (res.status != ikf::eMeasStatus::DISCARED) {
      insert_measurement(m, m.t_m);
      HistMeas.check_horizon();
    }
    return vec;
  }
}

ProcessMeasResult_t IDICOHandler::delegate_measurement(const MeasData &m) {
  ProcessMeasResult_t res;
  res.status = eMeasStatus::DISCARED;
  if (exists(m.id_sensor)) {
    if (id_dict[m.id_sensor]->enabled()) {
      res = id_dict[m.id_sensor]->delegate_measurement(m);
    }
  }
  res.t = m.t_m;
  res.observation_type = m.meas_type;
  return res;
}

ProcessMeasResult_vec_t IDICOHandler::redo_updates_from_t(const Timestamp &t) {
  remove_beliefs_from_t(t);
  Timestamp t_last;
  ProcessMeasResult_vec_t vec;
  if (HistMeas.get_latest_t(t_last)) {
    Logger::ikf_logger()->info("IDICOHandler::redo_updates_from_t() t=" + t.str() + ", t_last=" + t_last.str());
    if (t == t_last) {
      auto meas_arr = HistMeas.get_all_at_t(t);
      for (MeasData &m : meas_arr) {
        vec.push_back(this->delegate_measurement(m));
      }
    } else {
      HistMeas.foreach_between_t1_t2(t, t_last,
                                     [this, &vec](MeasData const &m) { vec.push_back(this->delegate_measurement(m)); });
    }
  }
  return vec;
}

ProcessMeasResult_vec_t IDICOHandler::redo_updates_after_t(const Timestamp &t) {
  remove_beliefs_after_t(t);
  Timestamp t_after, t_last;
  ProcessMeasResult_vec_t vec;
  if (HistMeas.get_after_t(t, t_after) && HistMeas.get_latest_t(t_last)) {
    Logger::ikf_logger()->info("IDICOHandler::redo_updates_after_t() t_after=" + t_after.str()
                               + ", t_last=" + t_last.str());
    if (t_after == t_last) {
      auto meas_arr = HistMeas.get_all_at_t(t_after);
      for (MeasData &m : meas_arr) {
        vec.push_back(this->delegate_measurement(m));
      }
    } else {
      HistMeas.foreach_between_t1_t2(t_after, t_last,
                                     [this, &vec](MeasData const &m) { vec.push_back(this->delegate_measurement(m)); });
    }
  } else {
    Logger::ikf_logger()->debug("IDICOHandler::redo_updates_after_t() t_after=" + t_after.str() + ": nothing to do...");
  }
  return vec;
}

void IDICOHandler::remove_beliefs_after_t(const Timestamp &t) {
  for (auto &elem : id_dict) {
    elem.second->remove_after_t(t);
  }
}

void IDICOHandler::remove_beliefs_from_t(const Timestamp &t) {
  for (auto &elem : id_dict) {
    elem.second->remove_from_t(t);
  }
}

std::shared_ptr<IIsolatedKalmanFilter> IDICOHandler::get(const size_t ID) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  auto it = id_dict.find(ID);
  if (it != id_dict.end()) {
    return it->second;
  }
  return std::shared_ptr<IIsolatedKalmanFilter>(nullptr);
}

void IDICOHandler::reset() {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  for (auto &elem : id_dict) {
    elem.second->reset();
  }
}

size_t IDICOHandler::get_propagation_sensor_ID(const size_t ID) { return m_PropSensor_ID; }

void IDICOHandler::set_propagation_sensor_ID(const size_t ID) { m_PropSensor_ID = ID; }

std::string IDICOHandler::get_type_by_ID(const size_t ID) {
  std::shared_ptr<IIsolatedKalmanFilter> pInst = get(ID);
  if (pInst) {
    return pInst->m_type;
  } else {
    return "";
  }
}

bool ikf::IDICOHandler::get_belief_at_t(const size_t ID, const Timestamp &t, pBelief_t &bel,
                                        const eGetBeliefStrategy type) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  if (exists(ID)) {
    return get(ID)->get_belief_at_t(t, bel, type);
  }
  return false;
}

bool ikf::IDICOHandler::get_beliefs_at_t(const std::vector<size_t> &IDs, const std::vector<eGetBeliefStrategy> &types,
                                         const Timestamp &t, std::map<size_t, pBelief_t> &beliefs) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  bool res = true;
  beliefs.clear();
  for (size_t idx = 0; idx < IDs.size(); idx++) {
    size_t ID = IDs.at(idx);
    auto type = types.at(idx);
    pBelief_t bel;
    bool success = get_belief_at_t(ID, t, bel, type);
    if (success) {
      beliefs.insert({ID, bel});
    }
    res &= success;
  }  // for-loop
  return res;
}

bool ikf::IDICOHandler::get_prop_meas_at_t(const size_t ID, const Timestamp &t, MeasData &m) {
  std::lock_guard<std::recursive_mutex> lk(m_mtx);

  if (exists(ID)) {
    return get(ID)->get_prop_meas_at_t(t, m);
  }
  return false;
}

bool IDICOHandler::is_order_violated(const MeasData &m) {
  if (m.obs_type != eObservationType::JOINT_OBSERVATION) {
    auto meas_arr = HistMeas.get_all_at_t(m.t_m);

    if (m.obs_type == eObservationType::PROPAGATION) {
      for (MeasData &m_ : meas_arr) {
        if (m_.obs_type == eObservationType::PRIVATE_OBSERVATION
            || m_.obs_type == eObservationType::JOINT_OBSERVATION) {
          return true;
        }
      }
    } else if (m.obs_type == eObservationType::PRIVATE_OBSERVATION) {
      for (MeasData &m_ : meas_arr) {
        if (m_.obs_type == eObservationType::JOINT_OBSERVATION) {
          return true;
        }
      }
    }
  }
  return false;
}

bool ikf::IDICOHandler::schedule_redo_updates_after_t(const Timestamp &t) {
  std::scoped_lock lk(m_mtx_histRUR);
  HistRedoUpdateRequest.insert(true, t);
  return true;
}

}  // namespace ikf
