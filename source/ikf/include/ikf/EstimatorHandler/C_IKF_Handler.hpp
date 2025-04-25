/******************************************************************************
 * FILENAME:     CollaborativeIKFHandler.hpp
 * PURPOSE:      Part of the ikf_lib
 * AUTHOR:       Roland Jung
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     03.02.2023
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
#ifndef COLLABORATIVE_IKF_HANDLER_HPP
#define COLLABORATIVE_IKF_HANDLER_HPP
#include <ikf/EstimatorHandler/ICSE_IKF_Handler.hpp>
#include <ikf/EstimatorHandler/IMultiAgentHandler.hpp>
#include <ikf/ikf_api.h>
#include <memory>
namespace ikf {

enum class IKF_API eRedoUpdateStrategy {
  EXACT = 0,           // trigger all direclty or indirectly post correlated agents: not implemented
  POSTCORRELATED = 1,  // trigger all directly post-correlated agents
  DISCARD = 2,         // measurement needs to be discared, since post-correlation exists
};

enum class IKF_API eOrderStrategy {
  STRICT = 0,   // propagation < private < joint
  RELAXED = 1,  // propagation < private = joint
};

std::string IKF_API to_string(eRedoUpdateStrategy const t);
std::string IKF_API to_string(eOrderStrategy const t);

// check ref: 'hidden' given overloaded virtual functions: https://stackoverflow.com/a/15295515
class IKF_API C_IKF_Handler : public ICSE_IKF_Handler {
public:
  C_IKF_Handler(MultiAgentHdl_ptr pAgentHdler, double const horizon_sec = 1.0);
  ~C_IKF_Handler() = default;

  // IsolatedKalmanFilter:
  virtual bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::MatrixXd& R,
                                 const Eigen::VectorXd& r, const Timestamp& t,
                                 const KalmanFilter::CorrectionCfg_t& cfg) override final;
  virtual bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::VectorXd& z,
                                 const Eigen::MatrixXd& R, const Timestamp& t,
                                 const KalmanFilter::CorrectionCfg_t& cfg) override final;
  using ICSE_IKF_Handler::apply_observation;  // w. function pointer h()
  virtual ProcessMeasResult_vec_t redo_updates_after_t(const Timestamp& t) override final;

protected:
  // ICSE_IKF_Handler:
  virtual ApplyObsResult_t apply_inter_agent_observation(
    const Eigen::MatrixXd& R, const Eigen::VectorXd& z, const Timestamp& t, IIsolatedKalmanFilter::h_joint const& h,
    std::vector<size_t> const& IDs, const KalmanFilter::CorrectionCfg_t& cfg,
    std::vector<IMultiAgentHandler::IDEstimator_t> const& remote_IDs,
    std::vector<IMultiAgentHandler::IDEstimator_t> const& local_IDs) override final;

  bool apply_inter_agent_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::MatrixXd& R,
                                     const Eigen::VectorXd& r, const Timestamp& t,
                                     const KalmanFilter::CorrectionCfg_t& cfg,
                                     std::vector<IMultiAgentHandler::IDEstimator_t> const& remote_IDs,
                                     std::vector<IMultiAgentHandler::IDEstimator_t> const& local_IDs);

  virtual bool discard_measurement(MeasData const& m) override final;
  virtual ProcessMeasResult_vec_t redo_updates_from_t(const Timestamp& t) override final;

  virtual std::map<size_t, pBelief_t> get_dict_bel(const std::map<size_t, Eigen::MatrixXd>& dict_H,
                                                   Timestamp const& t) override final;
  using ICSE_IKF_Handler::get_dict_bel;  // (const std::vector<size_t>& ids, Timestamp const& t)

  virtual Eigen::MatrixXd get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, Timestamp const& t) override final;
  virtual void set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd& Sigma_IJ,
                                 const Timestamp& t) override final;

  using IsolatedKalmanFilterHandler::apply_corrections_at_t;
  void apply_corrections_at_t(Eigen::MatrixXd& Sigma_apos, const std::map<size_t, pBelief_t>& dict_bel,
                              Timestamp const& t, const bool only_local_beliefs);

  std::set<size_t> get_correlated_IDs_after_t(Timestamp const& t);

  std::set<size_t> get_remote_correlated_IDs_after_t(Timestamp const& t);

  /////////////////////////////////////////////////////
  /// Interface for IKF handles to reprocess measurements
  virtual bool is_order_violated(MeasData const& m) override;

  eRedoUpdateStrategy mRedoStrategy{eRedoUpdateStrategy::POSTCORRELATED};
  eOrderStrategy mOrderStrategy{eOrderStrategy::STRICT};
};  // class CollaborativeIKFHandler

}  // namespace ikf

#endif  // COLLABORATIVE_IKF_HANDLER_HPP
