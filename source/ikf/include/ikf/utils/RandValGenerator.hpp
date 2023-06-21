/******************************************************************************
* FILENAME:     RandValGenerator.hpp
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
#ifndef RANDVALGENERATOR_HPP
#define RANDVALGENERATOR_HPP
#include <algorithm>
#include <ikf/ikf_api.h>
#include <random>

namespace ikf
{


// Test code:
//auto noise_gen = mmsf::GaussianNoiseGen(0, 1);
//std::cout << "Rand noise : ";
//for (int i = 0; i < 30; i++){
//  std::cout << std::to_string(noise_gen.rand_val()) << ",";
//}
//std::cout << std::endl;

///
/// \brief The NoiseGenerator class: allows to draw a sample for zero-mean white Gaussion distribution with a given "standard_deviation" and centered at "mean".
///
class IKF_API GaussianNoiseGen
{
public:
  static GaussianNoiseGen& instance();
  GaussianNoiseGen(GaussianNoiseGen const&) = delete;
  void operator=(GaussianNoiseGen const&) = delete;

  void seed(const uint64_t seed) const;
  double randn() const;
  std::vector<double> randn(size_t const N) const;

  inline double operator()() const {
    return distribution(gen);
  }

private:
  GaussianNoiseGen(double const mean = 0.0, double const standard_deviation=1.0);
  //std::random_device rd;//{};
  mutable std::mt19937 gen; //{rd()};
  mutable std::normal_distribution<double> distribution; //{0,node._std_range_m};
};


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


#endif // RANDVALGENERATOR_HPP
