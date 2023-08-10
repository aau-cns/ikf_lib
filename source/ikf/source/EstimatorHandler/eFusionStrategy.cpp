/******************************************************************************
 * FILENAME:     eFusionStrategy.cpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     10.08.2023
 *
 *  Copyright (C) 2023
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#include <ikf/EstimatorHandler/eFusionStrategy.hpp>

namespace ikf {

std::string to_string(const eFusionStrategy ot) {
  switch (ot) {
  case eFusionStrategy::IKF:
    return "IKF";
  case eFusionStrategy::DP:
    return "DP";
  case eFusionStrategy::CSE_IKF:
    return "CSE_IKF";
  default:
    break;
  }
  return "UNKNOWN";
}

eFusionStrategy to_eFusionStrategy(const std::string &str) {
  if (str == "IKF") {
    return eFusionStrategy::IKF;
  } else if (str == "DP") {
    return eFusionStrategy::DP;
  } else if (str == "CSE_IKF") {
    return eFusionStrategy::CSE_IKF;
  } else {
    return eFusionStrategy::UNKNOWN;
  }
}

}  // namespace ikf
