/******************************************************************************
 * FILENAME:     IMultiAgentHandler.cpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     09.08.2023
 *
 *  Copyright (C) 2023
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#include <ikf/EstimatorHandler/IMultiAgentHandler.hpp>

namespace ikf {

IMultiAgentHandler::IMultiAgentHandler(const IDAgent_t id) : m_AgentID(id) {}

IMultiAgentHandler::~IMultiAgentHandler() {}

IMultiAgentHandler::IDAgent_t IMultiAgentHandler::ID() const { return m_AgentID; }

std::vector<IMultiAgentHandler::IDEstimator_t> IMultiAgentHandler::get_estimator_IDs() {
  std::set<IDEstimator_t> IDs;
  for (auto const& e : dict_agents_ids) {
    for (EstimatorInfo_t const& info : e.second) {
      IDs.insert(info.ID);
    }
  }
  return std::vector<IDEstimator_t>(IDs.begin(), IDs.end());
}

std::vector<IMultiAgentHandler::IDAgent_t> IMultiAgentHandler::get_agent_IDs() {
  std::vector<IDAgent_t> IDs;
  IDs.reserve(dict_agents_ids.size());
  for (auto const& e : dict_agents_ids) {
    IDs.push_back(e.first);
  }
  return IDs;
}

IMultiAgentHandler::dict_id_type_t IMultiAgentHandler::get_types() {
  dict_id_type_t dict;
  for (auto const& e : dict_agents_ids) {
    for (EstimatorInfo_t const& info : e.second) {
      dict[info.ID] = info.type;
    }
  }
  return dict;
}

std::string IMultiAgentHandler::get_type_by_ID(const IDEstimator_t ID) {
  dict_id_type_t dict = get_types();
  if (dict.find(ID) != dict.end()) {
    return dict[ID];
  }
  return "";
}

IMultiAgentHandler::IDAgent_t IMultiAgentHandler::estimatorID2agentID(const IDEstimator_t ID_est) {
  if (dict_ID2Agent.find(ID_est) != dict_ID2Agent.end()) {
    return dict_ID2Agent[ID_est];
  }
  return 0;
}

bool IMultiAgentHandler::exists(const IDEstimator_t ID_est) {
  std::vector<IDEstimator_t> vec = get_estimator_IDs();
  if (std::find(vec.begin(), vec.end(), ID_est) != vec.end()) {
    return true;
  }
  return false;
}

std::ostream& operator<<(std::ostream& out, const IMultiAgentHandler& obj) {
  out << "IMultiAgentHandler:";
  out << " Agent_ID=[" << obj.m_AgentID << "]" << std::endl;
  for (auto const& e : obj.dict_agents_ids) {
    out << "*  other Agent_ID[" << e.first << "]: Sensor IDs: [";
    for (auto const& i : e.second) {
      out << "(" << i.ID << "/" << i.type << "),";
    }
    out << "]" << std::endl;
  }
  return out;
}

std::string IMultiAgentHandler::str() const {
  std::stringstream ss;
  ss << *this;
  return ss.str();
}

}  // namespace ikf
