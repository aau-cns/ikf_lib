/******************************************************************************
 * FILENAME:     IMMSF.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr-ait@github
 * MAIL:         <your mail address>
 * VERSION:      v0.0.0
 * CREATION:     23.01.2023
 *
 *  Copyright (C) 2023
 *  All rights reserved. See the LICENSE file for details.
 *
 *  References:
 *  [1] Roland Jung and Stephan Weiss, "Scalable Recursive Distributed Collaborative State Estimation for Aided
Inertial Navigation", Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), IEEE, Xi’an, 2021.
 ******************************************************************************/
#ifndef IIKF_HPP
#define IIKF_HPP
#include <ikf/ikf_api.h>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <ikf/container/TTimeHorizonBuffer.hpp>
#include <ikf/utils/RTVerification.hpp>
#include <ikf/Sensor/Estimate/IBelief.hpp>

#include <unordered_map>
namespace ikf {

class IInstanceHandler; // foward declaration

class IKF_API IMMSF {
public:
  IMMSF(std::shared_ptr<IInstanceHandler> ptr_hdl,
        std::string const &name,
        size_t ID = 0,
        double const horizon_sec=1.0);

  virtual std::string get_Name() const { return m_name; }
  virtual size_t get_ID() const { return m_ID; }
  virtual Timestamp current_t() const;
  virtual bool exist_belief_at_t(Timestamp const& t) const;
  virtual ptr_belief get_belief_at_t(Timestamp const& t) const;
  virtual bool get_belief_at_t(Timestamp const& t, ptr_belief& bel);
  virtual void set_belief_at_t(ptr_belief const& bel, Timestamp const&t);
  virtual bool correct_belief_at_t(Eigen::VectorXd const& mean_corr, Eigen::MatrixXd const& Sigma_apos,
                                   Timestamp const&t);
  virtual bool get_belief_before_t(Timestamp const&t, ptr_belief& bel, Timestamp &t_before);
  virtual Eigen::VectorXd get_mean_at_t(Timestamp const &t) const;
  virtual Eigen::MatrixXd get_Sigma_at_t(Timestamp const&t) const;
  virtual void reset();
  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void set_horizon(double const t_hor);
  virtual void check_horizon();
  ///// PURE VIRTUAL METHODS
  virtual bool apply_propagation(size_t const ID_I, Eigen::VectorXd const& mean_II,
                                 Eigen::MatrixXd const& Phi_II, Eigen::MatrixXd const& Q_II,
                                 Timestamp const& t_old, Timestamp const& t_new) = 0;
  virtual bool apply_private_observation(size_t const ID_I, Eigen::MatrixXd const& H_II,
                                         Eigen::MatrixXd const&R, Eigen::VectorXd const & r,
                                         Timestamp const& t) = 0;
  virtual bool apply_joint_observation(size_t const ID_I, size_t const ID_J, Eigen::MatrixXd const& H_II,
                                       Eigen::MatrixXd const& H_JJ, Eigen::MatrixXd const& R,
                                       Eigen::VectorXd const& r, Timestamp const& t) = 0;

  // TODO: vectorization of joint observation
  //virtual bool apply_joint_observation(std::vector<size_t> IDs, std::vector<Eigen::MatrixXd> const& H, Eigen::MatrixXd const& R, Eigen::VectorXd const& r, Timestamp const& t) = 0;

  //virtual bool apply_triple_observation(size_t const ID_I, size_t const ID_J, size_t const ID_K, Eigen::MatrixXd const& H_II, Eigen::MatrixXd const& H_JJ, Eigen::MatrixXd const& H_KK, Eigen::MatrixXd const& R, Eigen::VectorXd const& r, Timestamp const& t);
  //virtual bool apply_quad_observation(size_t const ID_I, size_t const ID_J, size_t const ID_K, size_t const ID_L, Eigen::MatrixXd const& H_II, Eigen::MatrixXd const& H_JJ, Eigen::MatrixXd const& H_KK, Eigen::MatrixXd const& H_LL, Eigen::MatrixXd const& R, Eigen::VectorXd const& r, Timestamp const& t);

protected:
  std::shared_ptr<IInstanceHandler> ptr_FC_hdl;
  TTimeHorizonBuffer<ptr_belief> HistBelief;
  size_t keep_n_elems = 4;
  std::string m_name = "unknown";
  size_t m_ID;
  double max_time_horizon_sec;
};




} // namespace ikf

#endif // IIKF_HPP
