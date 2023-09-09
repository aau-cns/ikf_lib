/******************************************************************************
 * FILENAME:     ProcessMeasResult_t.cpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     08.09.2023
 *
 *  Copyright (C) 2023
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#include <ikf/Estimator/ProcessMeasResult_t.hpp>

namespace ikf {

std::string to_string(const eMeasStatus s) {
  switch (s) {
  case eMeasStatus::REJECTED:
    return "REJECTED";
  case eMeasStatus::PROCESSED:
    return "PROCESSED";
  case eMeasStatus::OUTOFORDER:
    return "OUTOFORDER";
  case eMeasStatus::DISCARED:
    return "DISCARED";
  default:
    return "";
    break;
  }
}

ProcessMeasResult_t::ProcessMeasResult_t(const eMeasStatus &s) : status(s) {}

ProcessMeasResult_t::ProcessMeasResult_t(const eMeasStatus &s, const std::string &type)
  : status(s), observation_type(type) {}

}  // namespace ikf
