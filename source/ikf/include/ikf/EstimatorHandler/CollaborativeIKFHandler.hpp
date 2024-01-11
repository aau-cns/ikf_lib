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
#include <ikf/EstimatorHandler/IMultiAgentHandler.hpp>
#include <ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp>
#include <ikf/ikf_api.h>
#include <memory>
namespace ikf {

class IKF_API CollaborativeIKFHandler : public IsolatedKalmanFilterHandler {
public:
  CollaborativeIKFHandler(MultiAgentHdl_ptr pAgentHdler, double const horizon_sec = 1.0);
  ~CollaborativeIKFHandler() = default;

  virtual size_t get_propagation_sensor_ID(size_t const ID = 0) override;
  virtual std::string get_type_by_ID(size_t const ID = 0) override;

  virtual bool get_belief_at_t(size_t const ID, Timestamp const& t, pBelief_t& bel,
                               eGetBeliefStrategy const type = eGetBeliefStrategy::EXACT) override;

  virtual bool get_beliefs_at_t(std::vector<size_t> const& IDs, std::vector<eGetBeliefStrategy> const& types,
                                Timestamp const& t, std::map<size_t, pBelief_t>& beliefs) override;

  virtual bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::MatrixXd& R,
                                 const Eigen::VectorXd& r, const Timestamp& t,
                                 const KalmanFilter::CorrectionCfg_t& cfg) override;
  virtual bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::VectorXd& z,
                                 const Eigen::MatrixXd& R, const Timestamp& t,
                                 const KalmanFilter::CorrectionCfg_t& cfg) override;

protected:
  bool apply_inter_agent_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::MatrixXd& R,
                                     const Eigen::VectorXd& r, const Timestamp& t,
                                     const KalmanFilter::CorrectionCfg_t& cfg,
                                     std::vector<IMultiAgentHandler::IDEstimator_t> const& remote_IDs,
                                     std::vector<IMultiAgentHandler::IDEstimator_t> const& local_IDs);

  bool get_local_beliefs_and_FCC_at_t(std::vector<IMultiAgentHandler::IDEstimator_t> const& IDs,
                                      std::vector<IMultiAgentHandler::IDEstimator_t> const& ID_participants,
                                      Timestamp const& t,
                                      std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t>& beliefs,
                                      std::map<size_t, std::map<size_t, Eigen::MatrixXd>>& dict_FFC);

  Eigen::MatrixXd stack_Sigma_locally(const std::map<size_t, pBelief_t>& dict_bel, const Timestamp& t,
                                      std::map<size_t, std::map<size_t, Eigen::MatrixXd>>& dict_FFC);

  void split_Sigma_locally(Eigen::MatrixXd& Sigma, const std::map<size_t, pBelief_t>& dict_bel,
                           std::map<size_t, std::map<size_t, Eigen::MatrixXd>>& dict_FCC);

  virtual std::map<size_t, pBelief_t> get_dict_bel(const std::map<size_t, Eigen::MatrixXd>& dict_H,
                                                   Timestamp const& t) override;

  virtual Eigen::MatrixXd get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, Timestamp const& t) override;
  virtual void set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd& Sigma_IJ,
                                 const Timestamp& t) override;

  void apply_corrections_at_t(Eigen::MatrixXd& Sigma_apos, const std::map<size_t, pBelief_t>& dict_bel,
                              Timestamp const& t, const bool only_local_beliefs);

  MultiAgentHdl_ptr m_pAgentHandler;
};  // class CollaborativeIKFHandler

}  // namespace ikf

#endif  // COLLABORATIVE_IKF_HANDLER_HPP
