/******************************************************************************
* FILENAME:     IInstanceHandle.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Strategies/IInstanceHandle.hpp>
#include <ikf/Strategies/MMSF_IKF.hpp>

namespace ikf {

IInstanceHandle::IInstanceHandle(std::shared_ptr<IInstanceHandler> ptr_hld, const eMMSFTypes &strategy, const std::string &name, const size_t ID, double const horizon_sec) : m_ID(ID), m_name(name)
{

  switch(strategy) {
    case eMMSFTypes::IKF:
    {
      ptr_instance = std::make_shared<MMSF_IKF>(MMSF_IKF(ptr_hld, name, ID, horizon_sec));
    }
    default: ;

  }
}

IInstanceHandle::~IInstanceHandle() {}

inline size_t IInstanceHandle::get_ID() { return ptr_instance->get_ID(); }
inline std::string IInstanceHandle::get_Name() { return ptr_instance->get_Name(); }

Timestamp IInstanceHandle::current_t() const { return ptr_instance->current_t();}

bool IInstanceHandle::exist_belief_at_t(const Timestamp &t) const { return ptr_instance->exist_belief_at_t(t); }

ptr_belief IInstanceHandle::get_belief_at_t(const Timestamp &t) const {return ptr_instance->get_belief_at_t(t); }
inline std::shared_ptr<IMMSF> IInstanceHandle::get_IKF_hdl() { return ptr_instance; }

void IInstanceHandle::reset() {
  ptr_instance->reset();
}



} // ns mmsf
