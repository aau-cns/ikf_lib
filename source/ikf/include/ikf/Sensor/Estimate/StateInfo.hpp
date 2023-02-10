/******************************************************************************
* FILENAME:     StateInfo.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     30.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef STATEINFO_HPP
#define STATEINFO_HPP
#include <string>
#include <unordered_map>
#include <vector>
#include <map>
#include <ikf/ikf_api.h>

namespace ikf
{
class IKF_API StateInfo {
public:
  StateInfo(std::vector<std::string> const& types,
            std::vector<unsigned> const& dimensions,
            std::vector<std::string> const& units);

  void set_info(std::vector<std::string> const& types,
                std::vector<unsigned> const& dimensions,
                std::vector<std::string> const& units);
  void add(std::string const& type, unsigned const dimension, std::string const& unit, bool recalc_start_idx=true);
  bool exists(std::string const& type) const;
  std::vector<std::string> types() const;
  std::vector<std::string> units() const;
  std::vector<unsigned> dimensions() const;
  std::vector<unsigned> start_indicies() const;
  size_t dim() const;

private:
  size_t m_dim = 0;

  // TODO: the problem with maps is, that we loose the order the elements are added.
  std::unordered_map<std::string, unsigned> dict_dimensions;
  std::unordered_map<std::string, unsigned> dict_start_indicies;
  std::unordered_map<std::string, std::string> dict_units;
  // TODO: how to efficiently access Eigen::Vector and Eigen::Matrix? with StateInfo?
  // Sigma_p_gi = Sigma.block(start_idx, start_idx, rows, cols)
};


} // namespace ikf
#endif // STATEINFO_HPP
