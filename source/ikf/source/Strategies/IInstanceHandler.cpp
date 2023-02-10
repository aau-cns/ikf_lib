/******************************************************************************
* FILENAME:     IInstanceHandler.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Strategies/IInstanceHandler.hpp>
namespace ikf {

bool IInstanceHandler::add(std::shared_ptr<IInstanceHandle> ptr_instance) {
  if (ptr_instance) {
    size_t ID = ptr_instance->get_ID();
    std::string name =ptr_instance->get_Name();
    if ((ID != 0) && !name.empty()) {
      id_dict.emplace(ID, ptr_instance);
      name_dict.emplace(name, ptr_instance);
      return true;
    }
  }
  return false;
}

bool IInstanceHandler::remove(const size_t ID) {
  auto elem = get_by_id(ID);
  if (elem) {
    id_dict.erase(elem->get_ID());
    name_dict.erase(elem->get_Name());
    return true;
  }
  return false;
}

bool IInstanceHandler::remove(const std::string &name) {
  auto elem = get_by_name(name);
  if (elem) {
    id_dict.erase(elem->get_ID());
    name_dict.erase(elem->get_Name());
    return true;
  }
  return false;
}

bool IInstanceHandler::exists(const size_t ID) {
  return (id_dict.find(ID) != id_dict.end());
}

bool IInstanceHandler::exists(const std::string &name) {
  return (name_dict.find(name) != name_dict.end());
}

std::vector<size_t> IInstanceHandler::get_instance_ids() {
  std::vector<size_t> IDs;
  IDs.reserve(id_dict.size());
  for (auto const& elem : id_dict) {
    IDs.push_back(elem.first);
  }
  return IDs;
}

std::vector<std::string> IInstanceHandler::get_instance_names() {
  std::vector<std::string> IDs;
  IDs.reserve(name_dict.size());
  for (auto const& elem : name_dict) {
    IDs.push_back(elem.first);
  }
  return IDs;
}

std::shared_ptr<IInstanceHandle> IInstanceHandler::get_by_id(const size_t ID) {
  auto it = id_dict.find(ID);
  if (it != id_dict.end()) {
    return it->second;
  }
  return std::shared_ptr<IInstanceHandle>(nullptr);
}

std::shared_ptr<IInstanceHandle> IInstanceHandler::get_by_name(const std::string name) {
  auto it = name_dict.find(name);
  if (it != name_dict.end()) {
    return it->second;
  }
  return std::shared_ptr<IInstanceHandle>(nullptr);
}

std::shared_ptr<IMMSF> IInstanceHandler::get_IKF_hdl(const size_t ID) {
  auto ptr = get_by_id(ID);
  if (ptr) {
    return ptr->get_IKF_hdl();
  }
  return std::shared_ptr<IMMSF>(nullptr);
}

void IInstanceHandler::reset() {
  for (auto const& elem : name_dict) {
    elem.second->reset();
  }
}

} // namespace ikf
