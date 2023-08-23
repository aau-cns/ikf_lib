/******************************************************************************
 * FILENAME:     IMultiAgentHandler.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     09.08.2023
 *
 *  Copyright (C) 2023
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#ifndef IKF_IMULTIAGENTHANDLER_HPP
#define IKF_IMULTIAGENTHANDLER_HPP
#include "ikf/Estimator/IKalmanFilter.hpp"
#include <ikf/EstimatorHandler/IDICOHandler.hpp>
#include <ikf/ikf_api.h>
#include <map>
#include <vector>

namespace ikf {

class IKF_API IMultiAgentHandler {
public:
  typedef size_t IDAgent_t;
  typedef size_t IDEstimator_t;
  typedef std::map<IDAgent_t, std::vector<IDEstimator_t>> agents_ids_t;

  IMultiAgentHandler(IDAgent_t const id);
  virtual ~IMultiAgentHandler();

  IDAgent_t ID() const;

  virtual std::vector<IDEstimator_t> get_estimator_IDs();
  virtual std::vector<IDAgent_t> get_agent_IDs();

  IDAgent_t estimatorID2agentID(IDEstimator_t const ID_est);
  bool exists(IDEstimator_t const ID_est);

  virtual void request_agent_IDs() = 0;
  virtual bool refresh_lookup_table() = 0;
  virtual IDEstimator_t get_propagation_sensor_ID(IDEstimator_t const ID) = 0;

  virtual bool refresh_estimator_IDs(IDAgent_t const ID_agent) = 0;

  virtual bool get_belief_at_t(IDEstimator_t const ID_est, Timestamp const& t, pBelief_t& bel,
                               eGetBeliefStrategy const type = eGetBeliefStrategy::EXACT)
    = 0;

  // accelerates fetching the linearization points of measurement jacobians.
  virtual bool get_beliefs_at_t(std::vector<IDEstimator_t> const& IDs, std::vector<eGetBeliefStrategy> const& types,
                                Timestamp const& t, std::map<IDEstimator_t, pBelief_t>& beliefs)
    = 0;

  // accelerates stacking a priori joint beliefs.
  virtual bool get_beliefs_and_FCC_at_t(std::vector<IDEstimator_t> const& IDs,
                                        std::vector<IDEstimator_t> const& ID_participants, Timestamp const& t,
                                        std::map<IDEstimator_t, pBelief_t>& beliefs,
                                        std::map<size_t, std::map<size_t, Eigen::MatrixXd>>& dict_FFC)
    = 0;

  // accelerates splitting a posteriori beliefs.
  virtual bool set_beliefs_and_FCC_at_t(std::vector<IDEstimator_t> const& ID_participants, Timestamp const& t,
                                        std::map<IDEstimator_t, pBelief_t> const& beliefs,
                                        std::map<size_t, std::map<size_t, Eigen::MatrixXd>> const& dict_FFC,
                                        bool const calc_correction = true, bool const redo_update = true)
    = 0;

  virtual bool set_belief_at_t(IDEstimator_t const ID_est, Timestamp const& t, pBelief_t bel) = 0;

  virtual Eigen::MatrixXd get_CrossCovFact_IJ_at_t(const IDEstimator_t ID_I, const IDEstimator_t ID_J,
                                                   Timestamp const& t)
    = 0;

  virtual bool set_CrossCovFact_IJ_at_t(const IDEstimator_t ID_I, const IDEstimator_t ID_J,
                                        const Eigen::MatrixXd& FCC_IJ, const Timestamp& t)
    = 0;

  virtual bool apply_correction_at_t(const IDEstimator_t ID_I, Timestamp const& t, Eigen::MatrixXd const& Sigma_apri,
                                     Eigen::MatrixXd const Sigma_apos)
    = 0;

  virtual bool redo_updates_after_t(IDAgent_t const ID_agent, const Timestamp& t) = 0;

  void set_local_handler(pDICOHandler_t pHdler) { m_pLocalHandler = pHdler; }

  friend std::ostream& operator<<(std::ostream& out, const IMultiAgentHandler& obj);

  std::string str() const;

protected:
  agents_ids_t dict_agents_ids;
  IDAgent_t m_AgentID = 0;
  pDICOHandler_t m_pLocalHandler;
};

typedef std::shared_ptr<IMultiAgentHandler> MultiAgentHdl_ptr;

}  // namespace ikf

#endif  // IMULTIAGENTHANDLER_HPP
