/******************************************************************************
* FILENAME:     RandValGenerator.cpp
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
******************************************************************************/
#include <ikf/utils/RandValGenerator.hpp>
namespace ikf {

GaussianNoiseGen &GaussianNoiseGen::instance()
{
  static GaussianNoiseGen g(0.0, 1.0); // Guaranteed to be destroyed.
      // Instantiated on first use.
  return g;
}

GaussianNoiseGen::GaussianNoiseGen(const double mean, const double standard_deviation): gen((std::random_device())()), distribution(mean,standard_deviation)
{}

void GaussianNoiseGen::seed(const uint64_t seed) const { gen.seed(seed); }

double GaussianNoiseGen::randn() const {
  return distribution(gen);
}

std::vector<double> GaussianNoiseGen::randn(const size_t N) const {
  std::vector<double> xs(N);
  for(auto x = xs.begin(); x != xs.end(); x++) {
    *x = distribution(gen);
  }
  return xs;
}





} // ns mmsf
