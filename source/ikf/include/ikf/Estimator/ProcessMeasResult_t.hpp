/******************************************************************************
 * FILENAME:     ProcessMeasResult_t.hpp
 * PURPOSE:      Part of the ikf_lib
 * AUTHOR:       Roland Jung
 * MAIL:         <roland.jung@ieee.org>
 * VERSION:      v0.0.0
 * CREATION:     30.01.2023
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
#ifndef PROCESSMEASRESULT_T_HPP
#define PROCESSMEASRESULT_T_HPP
#include <Eigen/Dense>
#include <ikf/ikf_api.h>
#include <string>

namespace ikf {
enum class IKF_API eMeasStatus {
  REJECTED = 0,    // fused but rejected
  PROCESSED = 1,   // fused
  OUTOFORDER = 2,  // could not be fused. E.g., beliefs are missing -> OOE
  DISCARED = 3     // measurement needs to be discared
};

std::string IKF_API to_string(eMeasStatus const s);

struct IKF_API ProcessMeasResult_t {
  bool rejected = true;
  bool skipped = false;
  eMeasStatus status{eMeasStatus::REJECTED};
  Eigen::VectorXd residual;
  std::string observation_type;
  std::vector<size_t> ID_participants;

  ProcessMeasResult_t() = default;
  ProcessMeasResult_t(eMeasStatus const& s);

  friend std::ostream& operator<<(std::ostream& out, const ProcessMeasResult_t& obj) {
    out << "MeasResult: "
        << "status=:" << to_string(obj.status);
    out << ", residual:" << obj.residual << ", observation_type=" << obj.observation_type;
    out << ", num participants: " << obj.ID_participants.size();
    return out;
  }
};

}  // namespace ikf
#endif  // PROCESSMEASRESULT_T_HPP
