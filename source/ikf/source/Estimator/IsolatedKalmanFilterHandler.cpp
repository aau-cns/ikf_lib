/******************************************************************************
* FILENAME:     IsolatedKalmanFilterHandler.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
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




void IsolatedKalmanFilterHandler::redo_updates_after_t(std::vector<size_t> const& ID_participants, const Timestamp &t) {
  for (size_t const& elem : ID_participants) {
    if (exists(elem)) {
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
