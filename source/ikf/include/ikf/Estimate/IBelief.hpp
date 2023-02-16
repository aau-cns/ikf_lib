/******************************************************************************
* FILENAME:     Belief.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     25.01.2023
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
#ifndef MMSF_BELIEF_HPP
#define MMSF_BELIEF_HPP
#include <ikf/ikf_api.h>
#include <memory>
#include <iomanip>      // std::setprecision
#include <eigen3/Eigen/Eigen>
#include <ikf/Container/Timestamp.hpp>

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
  IBelief();
  IBelief(Eigen::VectorXd mean, Eigen::MatrixXd Sigma, Timestamp timestamp);
  virtual ~IBelief();
  virtual Eigen::VectorXd mean();
  virtual Eigen::MatrixXd Sigma();
  virtual void mean(const Eigen::VectorXd &vec);
  virtual void Sigma(const Eigen::MatrixXd &Cov);
  virtual bool set(const Eigen::VectorXd &mean, const Eigen::MatrixXd &Sigma);
  virtual IBelief& operator =(const Eigen::VectorXd &param);
  virtual IBelief &operator =(const Eigen::MatrixXd &param);
  // TODO: double check if operators are needed
  IBelief& operator= (const IBelief& param) = default;
  // Error-state size; should be DoF of the system process
  virtual size_t es_dim() const;
  // Nominal-state size; actual representation of the system process.
  virtual size_t ns_dim() const;
  virtual Timestamp const& timestamp() const;
  virtual void set_timestamp(Timestamp const& t);

  //virtual BeliefOptions const& options() const;
  //virtual void set_options(BeliefOptions const& o);

  ////////////////////////////////////////////////////////////
  //// PURE VIRTUAL:
  virtual std::shared_ptr<IBelief> clone() = 0;
  virtual std::shared_ptr<IBelief> interpolate(std::shared_ptr<IBelief> obj_a,
                                                       std::shared_ptr<IBelief> obj_b,
                                                       double const i) = 0; // % returns a new object!
  virtual void correct(Eigen::VectorXd const& dx) = 0; // inplace and accoring to the error definiton!
  virtual void correct(Eigen::VectorXd const& dx, const Eigen::MatrixXd& Sigma_apos) = 0; // inplace and accoring to the error definiton!
  //// PURE VIRTUAL:
  ////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////
  //// STATIC:
  static void apply_init_strategy(std::shared_ptr<IBelief>& bel_0, eInitStrategies const type, int const seed = 0);
  //// STATIC:
  ////////////////////////////////////////////////////////
  ///

  friend std::ostream& operator<< (std::ostream& out, const IBelief& obj) {
    out << "* IBelief:";
    out << std::left;
    out << " t=" << std::setw(16) << obj.m_timestamp.str();
    out << ", mean=" << std::setprecision(4) <<  obj.m_mean.transpose();
    out << ", diag(Sigma)=" << std::setprecision(4) << obj.m_Sigma.diagonal().transpose();
    out << std::internal;
    return out;
  }
protected:
  Eigen::VectorXd m_mean;
  Eigen::MatrixXd m_Sigma;
  size_t m_es_dim = 0;
  size_t m_ns_dim = 0;
  Timestamp m_timestamp;
  //BeliefOptions m_options;
};

typedef std::shared_ptr<IBelief> ptr_belief;

}
#endif // BELIEF_HPP
