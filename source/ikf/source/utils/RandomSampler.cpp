/******************************************************************************
* FILENAME:     RandomSampler.cpp
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
#include <ikf/utils/RandomSampler.hpp>
namespace ikf {



RandomSampler &RandomSampler::instance()
{
  static RandomSampler g; // Guaranteed to be destroyed.
      // Instantiated on first use.
  return g;

}

RandomSampler::RandomSampler() : gen((std::random_device())()) {}





} // ns ikf
