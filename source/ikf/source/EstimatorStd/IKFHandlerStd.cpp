/******************************************************************************
* FILENAME:     IKFHandlerStd.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/EstimatorStd/IKFHandlerStd.hpp>
namespace ikf {

bool IKFHandlerStd::add(ptr_IKF p_IKF) {
  if (!exists(p_IKF->ID())) {
    id_dict.emplace(p_IKF->ID(), p_IKF);
    return true;
  }
  return false;
}

bool IKFHandlerStd::remove(const size_t ID){
  auto elem = get(ID);
  if (elem) {
    id_dict.erase(elem->ID());
    return true;
  }
  return false;
}

bool IKFHandlerStd::exists(const size_t ID){
  return (id_dict.find(ID) != id_dict.end());
}

IKFHandlerStd::ptr_IKF IKFHandlerStd::get(const size_t ID){
  auto it = id_dict.find(ID);
  if (it != id_dict.end()) {
    return it->second;
  }
  return ptr_IKF(nullptr);
}



} // ns ikf
