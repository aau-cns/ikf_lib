/******************************************************************************
* FILENAME:     Belief.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     25.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef MMSF_BELIEF_HPP
#define MMSF_BELIEF_HPP
#include <ikf/ikf_api.h>
#include <ikf/Sensor/Estimate/StateInfo.hpp>
#include <eigen3/Eigen/Eigen>
#include <ikf/Sensor/Estimate/ISensorEstimate.hpp>
#include <ikf/container/Timestamp.hpp>
namespace ikf {


struct IKF_API BeliefOptions {
  bool is_fixed = false;
  bool do_fej   = false;
};

enum class IKF_API  eInitStrategies {
  None = 0,
  Random
};


class IKF_API IBelief {
public:
  virtual ~IBelief();
  virtual Eigen::VectorXd mean() = 0;
  virtual Eigen::MatrixXd Sigma() = 0;
  virtual void mean(Eigen::VectorXd const& vec) = 0;
  virtual void Sigma(Eigen::MatrixXd const& Cov) = 0;

  virtual bool set(Eigen::VectorXd const& mean, Eigen::MatrixXd const& Sigma) = 0;


  IBelief& operator= (const IBelief& param) = default;
  virtual IBelief& operator= (const Eigen::VectorXd& param) = 0;
  virtual IBelief& operator= (const Eigen::MatrixXd& param) = 0;

  virtual std::shared_ptr<IBelief> clone() = 0;
  virtual std::shared_ptr<IBelief> interpolate(std::shared_ptr<IBelief> obj_a,
                                                       std::shared_ptr<IBelief> obj_b,
                                                       double const i) = 0; // % returns a new object!
  virtual void correct(Eigen::VectorXd const& dx) = 0; // inplace and accoring to the error definiton!
  virtual void correct(Eigen::VectorXd const& dx, const Eigen::MatrixXd& Sigma_apos) = 0; // inplace and accoring to the error definiton!

  //virtual StateInfo const& state_info() const = 0;
  //virtual StateInfo const& es_state_info() const = 0;



  // Error-state size; should be DoF of the system process
  virtual size_t es_dim() const;
  // Nominal-state size; actual representation of the system process.
  virtual size_t ns_dim() const;
  virtual size_t unique_ID() const;
  virtual Timestamp const& timestamp() const;
  virtual void set_timestamp(Timestamp const& t);

  virtual BeliefOptions const& options() const;
  virtual void set_options(BeliefOptions const& o);

  static std::shared_ptr<IBelief> apply_init_strategy(std::shared_ptr<IBelief> const& bel_0, eInitStrategies const type, int const seed = 0);
private:
  size_t m_unique_ID = 0;
  size_t m_es_dim = 0;
  size_t m_ns_dim = 0;
  Timestamp m_timestamp;
  BeliefOptions m_options;
};

typedef std::shared_ptr<IBelief> ptr_belief;

}
#endif // BELIEF_HPP
