/******************************************************************************
 * FILENAME:     ICSE_IKF_Handler.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     27.06.2024
 *
 *  Copyright (C) 2024
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#ifndef ICSE_IKF_HANDLER_HPP
#define ICSE_IKF_HANDLER_HPP
#include <ikf/EstimatorHandler/IMultiAgentHandler.hpp>
#include <ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp>
#include <ikf/ikf_api.h>
#include <memory>

namespace ikf {

class IKF_API ICSE_IKF_Handler : public IsolatedKalmanFilterHandler {
public:
  ICSE_IKF_Handler(MultiAgentHdl_ptr pAgentHdler, double const horizon_sec = 1.0);
  ~ICSE_IKF_Handler() = default;

  virtual size_t get_propagation_sensor_ID(size_t const ID = 0) override final;
  virtual std::string get_type_by_ID(size_t const ID = 0) override final;

  virtual bool get_belief_at_t(size_t const ID, Timestamp const& t, pBelief_t& bel,
                               eGetBeliefStrategy const type = eGetBeliefStrategy::EXACT, bool const clone=false) override final;

  virtual bool get_beliefs_at_t(std::vector<size_t> const& IDs, std::vector<eGetBeliefStrategy> const& types,
                                Timestamp const& t, std::map<size_t, pBelief_t>& beliefs, bool const clone=false) override final;

  using IsolatedKalmanFilterHandler::apply_observation;  // bring all overloads into the scope
  virtual ApplyObsResult_t apply_observation(const Eigen::MatrixXd& R, const Eigen::VectorXd& z, const Timestamp& t,
                                             IIsolatedKalmanFilter::h_joint const& h, std::vector<size_t> const& IDs,
                                             const KalmanFilter::CorrectionCfg_t& cfg) override final;

protected:
  virtual ApplyObsResult_t apply_inter_agent_observation(
    const Eigen::MatrixXd& R, const Eigen::VectorXd& z, const Timestamp& t, IIsolatedKalmanFilter::h_joint const& h,
    std::vector<size_t> const& IDs, const KalmanFilter::CorrectionCfg_t& cfg,
    std::vector<IMultiAgentHandler::IDEstimator_t> const& remote_IDs,
    std::vector<IMultiAgentHandler::IDEstimator_t> const& local_IDs)
    = 0;

  std::set<IMultiAgentHandler::IDAgent_t> IDs_to_Agent_IDs(std::set<size_t> const& IDs);

  virtual std::map<size_t, pBelief_t> get_dict_bel(const std::vector<size_t>& ids, Timestamp const& t) override final;

  bool get_local_beliefs_and_FCC_at_t(std::vector<IMultiAgentHandler::IDEstimator_t> const& IDs,
                                      std::vector<IMultiAgentHandler::IDEstimator_t> const& ID_participants,
                                      Timestamp const& t,
                                      std::map<IMultiAgentHandler::IDEstimator_t, pBelief_t>& beliefs,
                                      std::map<size_t, std::map<size_t, Eigen::MatrixXd>>& dict_FFC);

  Eigen::MatrixXd stack_Sigma_locally(const std::map<size_t, pBelief_t>& dict_bel, const Timestamp& t,
                                      std::map<size_t, std::map<size_t, Eigen::MatrixXd>>& dict_FFC);

  void split_Sigma_locally(Eigen::MatrixXd& Sigma, const std::map<size_t, pBelief_t>& dict_bel,
                           std::map<size_t, std::map<size_t, Eigen::MatrixXd>>& dict_FCC);

  MultiAgentHdl_ptr m_pAgentHandler;
};  // class CollaborativeIKFHandler

}  // namespace ikf
#endif // ICSE_IKF_HANDLER_HPP
