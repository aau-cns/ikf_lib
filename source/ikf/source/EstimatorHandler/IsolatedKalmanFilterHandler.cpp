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

IsolatedKalmanFilterHandler::IsolatedKalmanFilterHandler(const bool handle_delayed, const double horizon_sec) : m_handle_delayed_meas(handle_delayed), HistMeas(horizon_sec), m_horzion_sec(horizon_sec) {
  if (handle_delayed) {
    std::cout << "IsolatedKalmanFilterHandler will handle delayed measurements, therefore call it's process_measurement method!" << std::endl;
  }
}

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

bool IsolatedKalmanFilterHandler::handle_delayed_meas() const { return m_handle_delayed_meas; }

double ikf::IsolatedKalmanFilterHandler::horizon_sec() const { return m_horzion_sec; }

bool ikf::IsolatedKalmanFilterHandler::insert_measurement(const MeasData &m, const Timestamp &t) {
  if(m_handle_delayed_meas) {
    HistMeas.insert(m, t);
  }
  return m_handle_delayed_meas;
}

void IsolatedKalmanFilterHandler::sort_measurements_from_t(const Timestamp &t) {
  Timestamp t_latest;
  if(HistMeas.get_latest_t(t_latest)) {
    TMultiHistoryBuffer<MeasData> meas = HistMeas.get_between_t1_t2(t, t_latest);

    if(!meas.empty()) {
      HistMeas.remove_after_t(t);
      HistMeas.remove_at_t(t);

      // first insert all PROPAGATION sorted
      meas.foreach([this](MeasData const& elem) {
        if (elem.obs_type == eObservationType::PROPAGATION && elem.obs_type != eObservationType::UNKNOWN ) {
          HistMeas.insert(elem, elem.t_m);
        }
      });

      // second insert all PRIVATE sorted
      meas.foreach([this](MeasData const& elem) {
        if (elem.obs_type == eObservationType::PRIVATE_OBSERVATION && elem.obs_type != eObservationType::UNKNOWN ) {
          HistMeas.insert(elem, elem.t_m);
        }
      });

      // third insert all JOINT sorted
      meas.foreach([this](MeasData const& elem) {
        if (elem.obs_type == eObservationType::JOINT_OBSERVATION && elem.obs_type != eObservationType::UNKNOWN ) {
          HistMeas.insert(elem, elem.t_m);
        }
      });
    }

  }
}

ProcessMeasResult_t IsolatedKalmanFilterHandler::process_measurement(const MeasData &m) {

  ProcessMeasResult_t res = reprocess_measurement(m);
  bool order_violated = is_order_violated(m);
  if (order_violated) {
    HistMeas.insert(m, m.t_m);
    sort_measurements_from_t(m.t_m);
    redo_updates_from_t(m.t_m);
  }
  else if (m_handle_delayed_meas) {
    if (!res.rejected && HistMeas.exist_after_t(m.t_m)) {
      redo_updates_after_t(m.t_m);
    }
    HistMeas.insert(m, m.t_m);
  }
  HistMeas.check_horizon();
  return res;
}

TMultiHistoryBuffer<MeasData> IsolatedKalmanFilterHandler::get_measurements_from_t(const Timestamp &t) {
  TMultiHistoryBuffer<MeasData>  hist_meas;
  // NOTE: in the concurrent case (simulatnous case) we can choose for a priorization of types.

  // first insert all PROPAGATION sorted
  for (auto& instance : id_dict) {
    TMultiHistoryBuffer<MeasData> meas = instance.second->get_measurements_from_t(t);
    if (!meas.empty()) {
      meas.foreach([&hist_meas](MeasData const& elem) {
        if (elem.obs_type == eObservationType::PROPAGATION && elem.obs_type != eObservationType::UNKNOWN ) {
          hist_meas.insert(elem, elem.t_m);
        }
      });
    }
  }

  // second insert all PRIVATE sorted
  for (auto& instance : id_dict) {
    TMultiHistoryBuffer<MeasData> meas = instance.second->get_measurements_from_t(t);
    if (!meas.empty()) {
      meas.foreach([&hist_meas](MeasData const& elem) {
        if (elem.obs_type == eObservationType::PRIVATE_OBSERVATION && elem.obs_type != eObservationType::UNKNOWN ) {
          hist_meas.insert(elem, elem.t_m);
        }
      });
    }
  }

  // third insert all JOINT sorted
  for (auto& instance : id_dict) {
    TMultiHistoryBuffer<MeasData> meas = instance.second->get_measurements_from_t(t);
    if (!meas.empty()) {
      meas.foreach([&hist_meas](MeasData const& elem) {
        if (elem.obs_type == eObservationType::JOINT_OBSERVATION && elem.obs_type != eObservationType::UNKNOWN ) {
          hist_meas.insert(elem, elem.t_m);
        }
      });
    }
  }
  return hist_meas;
}

TMultiHistoryBuffer<MeasData> IsolatedKalmanFilterHandler::get_measurements_after_t(const Timestamp &t) {
  TMultiHistoryBuffer<MeasData>  hist_meas;
  // NOTE: in the concurrent case (simulatnous case) we can choose for a priorization of types.

  // first insert all PROPAGATION sorted
  for (auto& elem : id_dict) {
    TMultiHistoryBuffer<MeasData> meas = elem.second->get_measurements_after_t(t);
    if (!meas.empty()) {
      meas.foreach([&hist_meas](MeasData const& elem) {
        if (elem.obs_type == eObservationType::PROPAGATION && elem.obs_type != eObservationType::UNKNOWN ) {
          hist_meas.insert(elem, elem.t_m);
        }
      });
    }
  }

  // second insert all PRIVATE sorted
  for (auto& elem : id_dict) {
    TMultiHistoryBuffer<MeasData> meas = elem.second->get_measurements_after_t(t);
    if (!meas.empty()) {
      meas.foreach([&hist_meas](MeasData const& elem) {
        if (elem.obs_type == eObservationType::PRIVATE_OBSERVATION && elem.obs_type != eObservationType::UNKNOWN ) {
          hist_meas.insert(elem, elem.t_m);
        }
      });
    }
  }

  // third insert all JOINT sorted
  for (auto& elem : id_dict) {
    TMultiHistoryBuffer<MeasData> meas = elem.second->get_measurements_after_t(t);
    if (!meas.empty()) {
      meas.foreach([&hist_meas](MeasData const& elem) {
        if (elem.obs_type == eObservationType::JOINT_OBSERVATION && elem.obs_type != eObservationType::UNKNOWN ) {
          hist_meas.insert(elem, elem.t_m);
        }
      });
    }
  }
  return hist_meas;
}

ProcessMeasResult_t IsolatedKalmanFilterHandler::reprocess_measurement(const MeasData &m) {
  ProcessMeasResult_t res;
  if(exists(m.id_sensor)) {
    res = id_dict[m.id_sensor]->reprocess_measurement(m);
  }
  return res;
}

bool IsolatedKalmanFilterHandler::redo_updates_from_t(const Timestamp &t) {
  remove_beliefs_from_t(t);
  Timestamp t_last;
  if (HistMeas.get_latest_t(t_last)) {
    std::cout << "IsolatedKalmanFilterHandler::redo_updates_from_t() t=" << t << ", t_last=" << t_last << std::endl;

    if (t == t_last) {
      auto vec = HistMeas.get_all_at_t(t);
      for (MeasData &m : vec) {
        this->reprocess_measurement(m);
      }
    }
    else {
      HistMeas.foreach_between_t1_t2(t, t_last, [this](MeasData const&m){ this->reprocess_measurement(m); });
    }
    return true;
  }
  return false;
}

bool IsolatedKalmanFilterHandler::redo_updates_after_t(const Timestamp &t) {
  remove_beliefs_after_t(t);
  Timestamp t_after, t_last;
  if (HistMeas.get_after_t(t, t_after) &&  HistMeas.get_latest_t(t_last)) {
    std::cout << "IsolatedKalmanFilterHandler::redo_updates_after_t() t_after=" << t_after << ", t_last=" << t_last << std::endl;
    if (t_after == t_last) {
      auto vec = HistMeas.get_all_at_t(t_after);
      for (MeasData &m : vec) {
        this->reprocess_measurement(m);
      }
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

void IsolatedKalmanFilterHandler::remove_beliefs_from_t(const Timestamp &t) {
  for (auto& elem : id_dict) {
    elem.second->remove_from_t(t);
  }
}

std::shared_ptr<IIsolatedKalmanFilter> IsolatedKalmanFilterHandler::get(const size_t ID) {
  auto it = id_dict.find(ID);
  if (it != id_dict.end()) {
    return it->second;
  }
  return std::shared_ptr<IIsolatedKalmanFilter>(nullptr);
}


void IsolatedKalmanFilterHandler::reset() {
  for (auto& elem : id_dict) {
    elem.second->reset();
  }
}

bool IsolatedKalmanFilterHandler::is_order_violated(const MeasData &m) {
  if (m.obs_type != eObservationType::JOINT_OBSERVATION) {
    auto meas_arr = HistMeas.get_all_at_t(m.t_m);

    if (m.obs_type == eObservationType::PROPAGATION) {
      auto meas_arr = HistMeas.get_all_at_t(m.t_m);
      for (MeasData & m_ : meas_arr) {
        if (m_.obs_type == eObservationType::PRIVATE_OBSERVATION ||
            m_.obs_type == eObservationType::JOINT_OBSERVATION ){
          return true;
        }
      }
    } else if  (m.obs_type == eObservationType::PRIVATE_OBSERVATION) {
      auto meas_arr = HistMeas.get_all_at_t(m.t_m);
      for (MeasData & m_ : meas_arr) {
        if (m_.obs_type == eObservationType::JOINT_OBSERVATION) {
          return true;
        }
      }
    }
  }
  return false;
}


} // namespace ikf
