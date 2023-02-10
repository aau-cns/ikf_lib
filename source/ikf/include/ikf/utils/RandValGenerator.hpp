/******************************************************************************
* FILENAME:     RandValGenerator.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     04.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
*
*
*  Singleton: https://stackoverflow.com/a/1008289
******************************************************************************/
#ifndef RANDVALGENERATOR_HPP
#define RANDVALGENERATOR_HPP
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


} // ns mmsf


#endif // RANDVALGENERATOR_HPP
