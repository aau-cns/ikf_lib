/******************************************************************************
* FILENAME:     IsolatedKalmanFilterHandler.cpp
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
#include <ikf/Estimator/IsolatedKalmanFilterHandler.hpp>

namespace ikf {

bool IsolatedKalmanFilterHandler::add(ptr_IKF p_IKF) {
  if (!exists(p_IKF->ID())) {
    // either one is handling delayed measurements!
    p_IKF->handle_delayed_meas(!m_handle_delayed_meas);
    id_dict.emplace(p_IKF->ID(), p_IKF);
    return true;
  }
  return false;
}

bool IsolatedKalmanFilterHandler::remove(const size_t ID) {
  auto elem = get(ID);
  if (elem) {
    id_dict.erase(elem->ID());
    return true;
  }
  return false;
}

bool IsolatedKalmanFilterHandler::exists(const size_t ID) {
  return (id_dict.find(ID) != id_dict.end());
}

std::vector<size_t> IsolatedKalmanFilterHandler::get_instance_ids() {
  std::vector<size_t> IDs;
  IDs.reserve(id_dict.size());
  for (auto const& elem : id_dict) {
    IDs.push_back(elem.first);
  }
  return IDs;
}

ProcessMeasResult_t IsolatedKalmanFilterHandler::process_measurement(const MeasData &m) {
  ProcessMeasResult_t res = reprocess_measurement(m);
  if (m_handle_delayed_meas) {
    if (!res.rejected && HistMeas.exist_after_t(m.t_m)) {
      redo_updates_after_t(m.t_m);
    }
    HistMeas.insert(m, m.t_m);
  }
  return res;
}

ProcessMeasResult_t IsolatedKalmanFilterHandler::reprocess_measurement(const MeasData &m) {
  ProcessMeasResult_t res;
  if(exists(m.id_sensor)) {
    res = id_dict[m.id_sensor]->process_measurement(m);
  }
  return res;
}

bool IsolatedKalmanFilterHandler::redo_updates_after_t(const Timestamp &t) {
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

void IsolatedKalmanFilterHandler::remove_beliefs_after_t(const Timestamp &t) {
  for (auto& elem : id_dict) {
    elem.second->remove_after_t(t);
  }
}

std::shared_ptr<IIsolatedKalmanFilter> IsolatedKalmanFilterHandler::get(const size_t ID) {
  auto it = id_dict.find(ID);
  if (it != id_dict.end()) {
    return it->second;
  }
  return std::shared_ptr<IIsolatedKalmanFilter>(nullptr);
}




void IsolatedKalmanFilterHandler::redo_updates_after_t(std::vector<size_t> const& ID_participants, const Timestamp &t) {
  for (size_t const& elem : ID_participants) {
    if (exists(elem)) {
      id_dict[elem]->redo_updates_after_t(t);
    }
  }
}

void IsolatedKalmanFilterHandler::redo_updates_after_t(size_t ID_master, const Timestamp &t) {
  for (size_t const& elem : get_instance_ids()) {
    if (exists(elem) && elem != ID_master) {
      id_dict[elem]->redo_updates_after_t(t);
    }
  }
}

void IsolatedKalmanFilterHandler::reset() {
  for (auto& elem : id_dict) {
    elem.second->reset();
  }
}


} // namespace ikf
