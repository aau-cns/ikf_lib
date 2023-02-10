/******************************************************************************
* FILENAME:     IInstanceHandler.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     30.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IINSTANCEHANDLE_HPP
#define IINSTANCEHANDLE_HPP
#include <ikf/ikf_api.h>
#include <ikf/Strategies/eMMSFTypes.hpp>
#include <ikf/Strategies/IMMSF.hpp>
#include <ikf/Sensor/Estimator/ProcessMeasResult_t.hpp>
#include <ikf/container/Timestamp.hpp>
#include <ikf/Sensor/Measurement/MeasData.hpp>
#include <ikf/Sensor/Estimate/IBelief.hpp>

namespace ikf
{

class IInstanceHandler; // forward declartion

// TODO: rename to IInstance
class IKF_API IInstanceHandle {
public:
  IInstanceHandle(std::shared_ptr<IInstanceHandler> ptr_hld, eMMSFTypes const& strategy, std::string const &name, size_t const ID = 0, const double horizon_sec=1.0);
  virtual ~IInstanceHandle();

  virtual bool enabled() { return m_enabled; }
  virtual void enabled(bool const val) { m_enabled = val; }
  virtual size_t get_ID();
  virtual std::string get_Name();
  virtual Timestamp current_t() const;
  virtual bool exist_belief_at_t(Timestamp const& t) const;
  virtual ptr_belief get_belief_at_t(Timestamp const& t) const;
  virtual std::shared_ptr<IMMSF> get_IKF_hdl();
  virtual void reset();




  ///////////////////////////////////////////////////////////////////////////
  /// pure virtual methods:
  virtual ProcessMeasResult_t process_measurement(MeasData const& m) = 0;


#ifdef SUPPORT_CSE
  // the pointer contains state specific information and is needed e.g. to linearize Jacobians.
  // transfer ownership with shared ptr!
  virtual bool get_apri_belief_and_fcc_at_t(Timestamp const& t, size_t ID_other,  ptr_belief ptr_bel, Eigen::MatrixXd& factorized_cross_cov) = 0;
  virtual bool set_apos_belief_and_fcc_at_t(Timestamp const& t, size_t ID_other, ptr_belief ptr_bel, Eigen::MatrixXd& factorized_cross_cov) = 0;
  virtual bool set_apos_belief_and_fcc_at_t(Timestamp const& t, size_t ID_other, Eigen::VectorXd const& mean_corr, Eigen::MatrixXd const& Sigma,  Eigen::MatrixXd const& factorized_cross_cov) = 0;
  // virtual bool redo_updates_after_t(Timestamp const& t) = 0;
  // virtual std::vector<size_t> get_correlated_IDs() const  = 0;
#endif

protected:
  size_t m_ID = 0;
  std::string m_name = {""};
  bool m_enabled = true;
  std::shared_ptr<IMMSF> ptr_instance;  // specifies, it the instance maintains a history of past measurements or not
};

typedef std::shared_ptr<IInstanceHandle> ptr_instance;



} // namespace ikf


#endif // IINSTANCEHANDLE_HPP
