/******************************************************************************
 * FILENAME:     DCI_IKF_Handler.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     26.06.2024
 *
 *  Copyright (C) 2024
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#ifndef CI_IKF_HANDLER_HPP
#define CI_IKF_HANDLER_HPP
#include <ikf/EstimatorHandler/ICSE_IKF_Handler.hpp>
#include <ikf/EstimatorHandler/IMultiAgentHandler.hpp>
#include <ikf/ikf_api.h>
#include <memory>
namespace ikf {

class IKF_API DCI_IKF_Handler : public ICSE_IKF_Handler {
public:
  DCI_IKF_Handler(MultiAgentHdl_ptr pAgentHdler, double const horizon_sec = 1.0);
  ~DCI_IKF_Handler() = default;

protected:
  virtual ApplyObsResult_t apply_inter_agent_observation(
    const Eigen::MatrixXd& R, const Eigen::VectorXd& z, const Timestamp& t, IIsolatedKalmanFilter::h_joint const& h,
    std::vector<size_t> const& IDs, const KalmanFilter::CorrectionCfg_t& cfg,
    std::vector<IMultiAgentHandler::IDEstimator_t> const& remote_IDs,
    std::vector<IMultiAgentHandler::IDEstimator_t> const& local_IDs) override final;

};  // class CollaborativeIKFHandler

}  // namespace ikf

#endif // CI_IKF_HANDLER_HPP
