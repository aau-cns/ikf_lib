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
#include <ikf/utils/lock_guard_timed.hpp>

namespace ikf {

IDICOHandler::IDICOHandler(const double horizon_sec)
  : HistMeas(horizon_sec),
    Hist_OOO_Flag(horizon_sec * 0.25),
    m_horzion_sec(horizon_sec),
    HistRedoUpdateRequest(horizon_sec * 0.5),
    mtx_timeout_ms(5ms)  // how long a threads waits to acess a locked state
{
  Logger::ikf_logger()->info("IDICOHandler will handle delayed measurements for all it's instances (centralized)!");
  Logger::ikf_logger()->info("IDICOHandler: m_horizon_sec=" + std::to_string(m_horzion_sec));
}

bool IDICOHandler::add(pIKF_t p_IKF) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    if (!exists(p_IKF->ID())) {
      // either one is handling delayed measurements!
      p_IKF->handle_delayed_meas(false);
      p_IKF->set_horizon(m_horzion_sec);
      id_dict.emplace(p_IKF->ID(), p_IKF);
      return true;
    }
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::add(): mutex FAILED");
  }
  return false;
}

bool IDICOHandler::remove(const size_t ID) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    auto elem = get(ID);
    if (elem) {
      id_dict.erase(elem->ID());
      return true;
    }
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::remove(): mutex FAILED");
  }
  return false;
}

bool IDICOHandler::exists(const size_t ID) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    return (id_dict.find(ID) != id_dict.end());
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::exists(): mutex FAILED");
  }
  return false;
}

std::vector<size_t> IDICOHandler::get_instance_ids() {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    std::vector<size_t> IDs;
    IDs.reserve(id_dict.size());
    for (auto const &elem : id_dict) {
      IDs.push_back(elem.first);
    }
    return IDs;
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::get_instance_ids(): mutex FAILED");
  }
  return std::vector<size_t>();
}

std::map<size_t, std::string> IDICOHandler::get_instance_types() {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    std::map<size_t, std::string> types;
    for (auto const &elem : id_dict) {
      types[elem.first] = elem.second->m_type;
    }
    return types;
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::get_instance_types(): mutex FAILED");
  }
  return std::map<size_t, std::string>();
}

double ikf::IDICOHandler::horizon_sec() const { return m_horzion_sec; }

void IDICOHandler::set_horizon(const double t_hor) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    m_horzion_sec = t_hor;
    for (auto const &elem : id_dict) {
      elem.second->set_horizon(m_horzion_sec);
    }
    HistMeas.set_horizon(m_horzion_sec);
    Hist_OOO_Flag.set_horizon(m_horzion_sec * 0.5);
    HistRedoUpdateRequest.set_horizon(m_horzion_sec * 0.5);
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::set_horizon(): mutex FAILED");
  }
}

bool IDICOHandler::reset_buffer() {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    HistMeas.clear();
    Hist_OOO_Flag.clear();
    HistRedoUpdateRequest.clear();
    return true;
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::reset_buffer(): mutex FAILED");
    return false;
  }
}

bool ikf::IDICOHandler::insert_measurement(const MeasData &m, const Timestamp &t) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    HistMeas.insert(m, t);
    return true;
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::insert_measurement(): mutex FAILED");
    return false;
  }
}

void IDICOHandler::print_HistMeas(std::ostream &out, size_t max) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    size_t cnt = 0;
    out << "|\t t_p \t |\t t_m \t |\t ID \t |\t type \t|" << std::endl;
    HistMeas.foreach ([&cnt, max, &out](ikf::MeasData const &i) {
      if (cnt < max) {
        out << "|\t " << std::setprecision(4) << i.t_p.to_sec() << " \t |\t " << std::setprecision(4) << i.t_m.to_sec()
            << " \t |\t " << i.id_sensor << " \t |\t" << ikf::to_string(i.obs_type) << "\t |" << std::endl;
      }
      cnt++;
    });
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::print_HistMeas(): mutex FAILED");
  }
}

void IDICOHandler::sort_measurements_from_t(const Timestamp &t) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
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
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::sort_measurements_from_t(): mutex FAILED");
  }
}

ProcessMeasResult_vec_t IDICOHandler::process_measurement(const MeasData &m) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    if (!exists(m.id_sensor)) {
      return ProcessMeasResult_vec_t({ProcessMeasResult_t(eMeasStatus::DISCARED)});
    }

    Timestamp t_latest;
    if (HistMeas.get_latest_t(t_latest)) {
      if (t_latest.to_sec() - m.t_m.to_sec() >= HistMeas.horizon()) {
        ikf::Logger::ikf_logger()->warn("IDICOHandler::process_measurement(): measurement is too much delayed: "
                                        + m.str_short());
        return ProcessMeasResult_vec_t({ProcessMeasResult_t(eMeasStatus::DISCARED)});
      }
    }

    // if there are open request, use the oldest one, before processing the new measurmeent
    HistRedoUpdateRequest.check_horizon_from_t(t_latest);
    if (HistRedoUpdateRequest.size()) {
      Timestamp t_oldest;
      bool after = true;
      {
        std::scoped_lock lk(m_mtx_histRUR);
        HistRedoUpdateRequest.get_oldest_t(t_oldest);
        HistRedoUpdateRequest.get_oldest(after);
        HistRedoUpdateRequest.clear();
      }

      // make sure that the request does not delete
      Timestamp t_oldest_meas;
      if (HistMeas.get_oldest_t(t_oldest_meas) && t_oldest_meas < t_oldest) {
        if (after) {
          IDICOHandler::redo_updates_after_t(t_oldest);  // restrict overloading
          ikf::Logger::ikf_logger()->info(
            "IDICOHandler::process_measurement(): requested redo update AFTER t=" + t_oldest.str() + " processed!");
        } else {
          IDICOHandler::redo_updates_from_t(t_oldest);  // restrict overloading
          ikf::Logger::ikf_logger()->info(
            "IDICOHandler::process_measurement(): requested redo update FROM t=" + t_oldest.str() + " processed!");
        }
      } else {
        ikf::Logger::ikf_logger()->warn(
          "IDICOHandler::process_measurement(): redo upate request is too much delayed, thus ignored!");
      }
    }

    if (discard_measurement(m)) {
      ikf::Logger::ikf_logger()->warn("IDICOHandler::process_measurement(): measurement " + m.str_short()
                                      + " needs to be discarded");
      return ProcessMeasResult_vec_t({ProcessMeasResult_t(eMeasStatus::DISCARED)});
    }

    // concurrent order
    bool order_violated = is_order_violated(m);

    Hist_OOO_Flag.check_horizon_from_t(m.t_m);
    bool ooo_exists = (Hist_OOO_Flag.exist_before_t(m.t_m) || Hist_OOO_Flag.exist_at_t(m.t_m));

    if (order_violated || ooo_exists) {
      ikf::Timestamp t_start = m.t_m;

      insert_measurement(m, m.t_m);
      HistMeas.check_horizon();

      if (ooo_exists) {
        // ikf::Logger::ikf_logger()->debug("IDICOHandler::process_measurement(): OOO measurement exists!");
        ikf::Timestamp t_oldest;
        if (Hist_OOO_Flag.get_oldest_t(t_oldest) && t_oldest < m.t_m) {
          t_start = t_oldest;
        }
        Hist_OOO_Flag.clear();
      }
      sort_measurements_from_t(t_start);

      auto vec = redo_updates_from_t(t_start);  // permit overloading
      return vec;
    } else {
      ProcessMeasResult_t res = delegate_measurement(m);
      ProcessMeasResult_vec_t vec({res});
      if (res.status == eMeasStatus::OUTOFORDER) {
        Hist_OOO_Flag.insert(true, m.t_m);
      }
      if (res.status != ikf::eMeasStatus::DISCARED) {
        insert_measurement(m, m.t_m);
        HistMeas.check_horizon();
      }  // else: DISCARD
      return vec;
    }
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::process_measurement(): mutex FAILED");
    return ProcessMeasResult_vec_t({ProcessMeasResult_t(eMeasStatus::OUTOFORDER)});
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
  res.meas_type = m.meas_type;
  return res;
}

ProcessMeasResult_vec_t IDICOHandler::redo_updates_from_t(const Timestamp &t) {
  remove_beliefs_from_t(t);
  Hist_OOO_Flag.clear();
  Timestamp t_last;
  ProcessMeasResult_vec_t vec;
  if (HistMeas.get_latest_t(t_last)) {
    double delta_t = t_last.to_sec() - t.to_sec();
    Logger::ikf_logger()->info("IDICOHandler::redo_updates_from_t(): t=" + t.str() + ", t_last=" + t_last.str()
                               + ", dt=" + std::to_string(delta_t));
    if (t == t_last) {
      auto meas_arr = HistMeas.get_all_at_t(t);
      for (MeasData &m : meas_arr) {
        // NOTE: if we have already spotted an OOO measurement, then just contiue with PROGAGATION measrements:
        if (Hist_OOO_Flag.empty() || m.obs_type == ikf::eObservationType::PROPAGATION) {
          auto res = this->delegate_measurement(m);
          vec.push_back(res);
          if (res.status == ikf::eMeasStatus::OUTOFORDER) {
            Hist_OOO_Flag.insert(true, m.t_m);
          }
        } else {
          Hist_OOO_Flag.insert(true, m.t_m);
        }
      }
    } else {
      HistMeas.foreach_between_t1_t2(t, t_last, [this, &vec](MeasData const &m) {
        if (Hist_OOO_Flag.empty() || m.obs_type == ikf::eObservationType::PROPAGATION) {
          auto res = this->delegate_measurement(m);
          vec.push_back(res);
          if (res.status == ikf::eMeasStatus::OUTOFORDER) {
            Hist_OOO_Flag.insert(true, m.t_m);
          }
        } else {
          Hist_OOO_Flag.insert(true, m.t_m);
        }
      });
    }
    Logger::ikf_logger()->info("IDICOHandler::redo_updates_from_t(): (num={:d}, OOO={:d}) DONE!", vec.size(),
                               Hist_OOO_Flag.size());
  }
  return vec;
}

ProcessMeasResult_vec_t IDICOHandler::redo_updates_after_t(const Timestamp &t) {
  remove_beliefs_after_t(t);
  Hist_OOO_Flag.clear();
  Timestamp t_after, t_last;
  ProcessMeasResult_vec_t vec;
  if (HistMeas.get_after_t(t, t_after) && HistMeas.get_latest_t(t_last)) {
    double delta_t = t_last.to_sec() - t_after.to_sec();
    Logger::ikf_logger()->info("IDICOHandler::redo_updates_after_t(): t_after=" + t_after.str()
                               + ", t_last=" + t_last.str() + ", dt=" + std::to_string(delta_t));

    if (t_after == t_last) {
      auto meas_arr = HistMeas.get_all_at_t(t_after);
      for (MeasData &m : meas_arr) {
        if (Hist_OOO_Flag.empty() || m.obs_type == ikf::eObservationType::PROPAGATION) {
          auto res = this->delegate_measurement(m);
          vec.push_back(res);
          if (res.status == ikf::eMeasStatus::OUTOFORDER) {
            Hist_OOO_Flag.insert(true, m.t_m);
          }
        } else {
          Hist_OOO_Flag.insert(true, m.t_m);
        }
      }
    } else {
      HistMeas.foreach_between_t1_t2(t_after, t_last, [this, &vec](MeasData const &m) {
        if (Hist_OOO_Flag.empty() || m.obs_type == ikf::eObservationType::PROPAGATION) {
          auto res = this->delegate_measurement(m);
          vec.push_back(res);
          if (res.status == ikf::eMeasStatus::OUTOFORDER) {
            Hist_OOO_Flag.insert(true, m.t_m);
          }
        } else {
          Hist_OOO_Flag.insert(true, m.t_m);
        }
      });
    }
    Logger::ikf_logger()->info("IDICOHandler::redo_updates_after_t(): (num={:d}, OOO={:d}) DONE!", vec.size(),
                               Hist_OOO_Flag.size());
  } else {
    Logger::ikf_logger()->debug("IDICOHandler::redo_updates_after_t() t_after=" + t.str() + ": nothing to do...");
  }
  return vec;
}

void IDICOHandler::remove_beliefs_after_t(const Timestamp &t) {
  for (auto &elem : id_dict) {
    // avoid deleting all beliefs from an instance
    if (elem.second->exist_belief_at_t(t) || elem.second->exist_belief_before_t(t)) {
      elem.second->remove_after_t(t);
    }
  }
}

void IDICOHandler::remove_beliefs_from_t(const Timestamp &t) {
  for (auto &elem : id_dict) {
    // avoid deleting all beliefs from an instance
    if (elem.second->exist_belief_before_t(t)) {
      elem.second->remove_from_t(t);
    }
  }
}

std::shared_ptr<IIsolatedKalmanFilter> IDICOHandler::get(const size_t ID) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    auto it = id_dict.find(ID);
    if (it != id_dict.end()) {
      return it->second;
    }
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::get(): mutex FAILED");
  }
  return std::shared_ptr<IIsolatedKalmanFilter>(nullptr);
}

bool IDICOHandler::reset() {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    for (auto &elem : id_dict) {
      elem.second->reset();
    }
    return true;
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::reset(): mutex FAILED");
    return false;
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

bool IDICOHandler::set_belief_at_t(const size_t ID, pBelief_t &bel, const Timestamp &t) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    if (exists(ID)) {
      get(ID)->set_belief_at_t(bel, t);
      return true;
    }
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::set_belief_at_t(): mutex FAILED");
  }
  return false;
}

bool IDICOHandler::get_belief_at_t(const size_t ID, const Timestamp &t, pBelief_t &bel, const eGetBeliefStrategy type) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    if (exists(ID)) {
      return get(ID)->get_belief_at_t(t, bel, type);
    }
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::get_belief_at_t(): mutex FAILED");
  }
  return false;
}

bool IDICOHandler::get_beliefs_at_t(const std::vector<size_t> &IDs, const std::vector<eGetBeliefStrategy> &types,
                                    const Timestamp &t, std::map<size_t, pBelief_t> &beliefs) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
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
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::get_beliefs_at_t(): mutex FAILED");
  }
  return false;
}

Eigen::MatrixXd IDICOHandler::get_CrossCovFact_at_t(const size_t ID_I, const Timestamp &t, const size_t ID_J) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    if (exists(ID_I)) {
      return get(ID_I)->get_CrossCovFact_at_t(t, ID_J);
    } else {
      return Eigen::MatrixXd();
    }
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::get_beliefs_at_t(): mutex FAILED");
  }
  return Eigen::MatrixXd();
}

bool IDICOHandler::set_CrossCovFact_at_t(const size_t ID_I, const Timestamp &t, const size_t ID_J,
                                         const Eigen::MatrixXd &FFC_IJ) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    if (exists(ID_I)) {
      get(ID_I)->set_CrossCovFact_at_t(t, ID_J, FFC_IJ);
      return true;
    }
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::set_CrossCovFact_at_t(): mutex FAILED");
  }
  return false;
}

bool IDICOHandler::apply_correction_at_t(const size_t ID, const Timestamp &t, const Eigen::MatrixXd &Factor) {
  ikf::lock_guard_timed<std::recursive_timed_mutex> lock(m_mtx, mtx_timeout_ms);
  if (lock.try_lock()) {
    if (exists(ID)) {
      return get(ID)->apply_correction_at_t(t, Factor);
    }
  } else {
    ikf::Logger::ikf_logger()->error("IDICOHandler::apply_correction_at_t(): mutex FAILED");
  }
  return false;
}

bool IDICOHandler::is_order_violated(const MeasData &m) {
  // if an measurement after the current one exists...
  if(HistMeas.exist_after_t(m.t_m)) {
    return true;
  }

  // if we have concurrent measurements, propagation before private; private before joint
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

bool IDICOHandler::discard_measurement(const MeasData &m) {
  return false;
}

bool IDICOHandler::schedule_redo_update(const Timestamp &t, const bool after) {
  std::scoped_lock lk(m_mtx_histRUR);
  HistRedoUpdateRequest.insert(after, t);
  HistRedoUpdateRequest.check_horizon();
  return true;
}

}  // namespace ikf
