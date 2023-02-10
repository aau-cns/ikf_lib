/******************************************************************************
* FILENAME:     StateInfo.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Sensor/Estimate/StateInfo.hpp>
#include <ikf/utils/RTVerification.hpp>

namespace ikf {

StateInfo::StateInfo(const std::vector<std::string> &types,
                     const std::vector<unsigned int> &dimensions,
                     const std::vector<std::string> &units) {
  set_info(types, dimensions, units);
}

void StateInfo::set_info(const std::vector<std::string> &types,
                         const std::vector<unsigned int> &dimensions,
                         const std::vector<std::string> &units){
  bool res = (types.size() == dimensions.size()) && (types.size() == units.size());
  RTV_EXPECT_TRUE_MSG(res, "Dimensions must fit!");
  if (res) {
    for(size_t i = 0; i < types.size(); i++)
    {
      add(types.at(i), dimensions.at(i), units.at(i), false);
    }
  }

}

void StateInfo::add(const std::string &type, const unsigned int dimension,
                    const std::string &unit, bool recalc_start_idx){
  bool existing = exists(type);
  RTV_EXPECT_FALSE_MSG(existing, "Type already exists: " + type);
  if (!existing) {
    dict_dimensions[type] = dimension;
    dict_start_indicies[type] = m_dim;
    m_dim += dimension;
    dict_units[type] = unit;
  }
}

bool StateInfo::exists(const std::string &type) const {
  return (dict_dimensions.find(type) != dict_dimensions.end());
}

std::vector<std::string> StateInfo::types() const {
  std::vector<std::string> types;
  types.reserve(dict_dimensions.size());
  for (auto const& elem : dict_dimensions)
  {
    types.push_back(elem.first);
  }
  return types;
}

std::vector<std::string> StateInfo::units() const {
  std::vector<std::string> types;
  types.reserve(dict_units.size());
  for (auto const& elem : dict_units)
  {
    types.push_back(elem.second);
  }
  return types;
}

std::vector<unsigned int> StateInfo::dimensions() const {
  std::vector<unsigned> types;
  types.reserve(dict_dimensions.size());
  for (auto const& elem : dict_dimensions)
  {
    types.push_back(elem.second);
  }
  return types;
}

std::vector<unsigned int> StateInfo::start_indicies() const {
  std::vector<unsigned> types;
  for (auto const& elem : dict_start_indicies)
  {
    types.push_back(elem.second);
  }
  return types;
}

size_t StateInfo::dim() const {
  return m_dim;
}


} // ns mmsf
