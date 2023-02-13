/******************************************************************************
* FILENAME:     IsolatedKalmanFilterHandler.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     03.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Estimator/IsolatedKalmanFilterHandler.hpp>

namespace ikf {

ProcessMeasResult_t IsolatedKalmanFilterHandler::process_measurement(const MeasData &m) {
  ProcessMeasResult_t res;

  if (exists(m.id_sensor)) {
    ptr_IKF p_instance = get(m.id_sensor);
    if (p_instance->enabled()) {
      res = p_instance->process_measurement(m);
    } else {
      res.skipped = true;
    }
    if (!res.skipped && !res.rejected && HistMeas.size()) {
      Timestamp t_oldest, t_latest;
      HistMeas.get_oldest_t(t_oldest);
      HistMeas.get_latest_t(t_latest);

      if(m.t_m < t_oldest) {
        std::cout << "Sensor measurement for " << p_instance->name() << ":Too much delay" << std::endl;
      }
      else if(m.t_m > t_latest) {
        redo_updates_after_t(m.t_m);
      }
      HistMeas.insert(m, m.t_m);
    }
    else if(!res.skipped) {
      HistMeas.insert(m, m.t_m);
    }
  }
  return res;
}

void IsolatedKalmanFilterHandler::redo_updates_after_t(const Timestamp &t) {
  remove_after_t(t);
  Timestamp t_latest;
  if(HistMeas.get_latest_t(t_latest)) {

    HistMeas.foreach_between_t1_t2(t, t_latest, [this](MeasData const& m) {
      ptr_IKF p_inst = this->id_dict.at(m.id_sensor);
      p_inst->process_measurement(m);
    });
  }
}

void IsolatedKalmanFilterHandler::remove_after_t(const Timestamp &t) {
  for (auto const& elem : this->id_dict) {
    elem.second->remove_after_t(t);
  }
}

void IsolatedKalmanFilterHandler::disp_measurement_hist()
{
  HistMeas.foreach([](MeasData const& m) {
    std::cout << m << std::endl;
  });
}

bool IsolatedKalmanFilterHandler::add(ptr_IKF p_IKF) {
  if (!exists(p_IKF->ID())) {
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



} // namespace ikf
