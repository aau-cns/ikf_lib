/******************************************************************************
* FILENAME:     RandomSampler.hpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     04.02.2023
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
*
* References:
*  Singleton: https://stackoverflow.com/a/1008289
******************************************************************************/
#ifndef IKF_RANDOMSAMPLER_HPP
#define IKF_RANDOMSAMPLER_HPP
#include <ikf/ikf_api.h>
#include <algorithm>
#include <random>

namespace ikf
{



class IKF_API RandomSampler
{
public:
  static RandomSampler& instance();
  RandomSampler(RandomSampler const&) = delete;
  void operator=(RandomSampler const&) = delete;

  template<typename T>
  std::vector<T> sample(std::vector<T> const& vec, size_t const N) const{
    std::vector<T> out;
    out.reserve(N);
    std::sample(vec.begin(), vec.end(), std::back_inserter(out),
                N, gen);

    return out;
  }


private:
  RandomSampler();
  //std::random_device rd;//{};
  mutable std::mt19937 gen; //{rd()};
  mutable std::normal_distribution<double> distribution; //{0,node._std_range_m};
};

} // ns mmsf


#endif // IKF_RANDOMSAMPLER_HPP
