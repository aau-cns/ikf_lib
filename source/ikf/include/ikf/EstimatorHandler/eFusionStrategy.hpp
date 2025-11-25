/******************************************************************************
 * FILENAME:     eFusionStrategy.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     10.08.2023
 *
 *  Copyright (C) 2023
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#ifndef IKF_EFUSIONSTRATEGY_HPP
#define IKF_EFUSIONSTRATEGY_HPP
#include <ikf/ikf_api.h>
#include <string>

namespace ikf {

enum class IKF_API eFusionStrategy {
  UNKNOWN = 0,
  IKF,
  DP,    // Distributed Propagation - Centralized Equvivalent
  C_IKF, // Collaborative IKF
  DCI_IKF, // Distributed Covariance Intersection-based IKF
  SKF, // Smidt-Kalman Filter.
  //
};

std::string IKF_API to_string(eFusionStrategy const ot);
eFusionStrategy IKF_API to_eFusionStrategy(std::string const& str);

}  // namespace ikf

#endif // EFUSIONSTRATEGY_HPP
