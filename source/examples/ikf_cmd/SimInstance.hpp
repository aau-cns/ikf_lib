/******************************************************************************
* FILENAME:     SimInstance.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef SIMINSTANCE_HPP
#define SIMINSTANCE_HPP
#include "Trajectory.hpp"
#include "LinearIKF.hpp"
#include <ikf/Estimate/LinearBelief.hpp>


class SimInstance {
public:
  SimInstance(const Eigen::MatrixXd &F, const Eigen::MatrixXd &G, const Eigen::MatrixXd &Q,
              const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
              size_t const ID, double const dt, double const D, double const omega,
              double const std_dev_p, double const std_dev_a, double const std_dev_p_rel,
              std::shared_ptr<ikf::IKFHandlerStd> ptr_Handler) : ID(ID), dt(dt), std_dev_p(std_dev_p), std_dev_a(std_dev_a), std_dev_p_rel(std_dev_p_rel), ptr_IKF(new LinearIKF(ptr_Handler, ID)) {

    //double const omega = 0.4;
    double const omega_0 = 0.4;
    ptr_IKF->define_system(F, G, Q, H, R, dt);
    traj.generate_sine(dt, D, omega, omega_0*ID, 1, ID);

    traj_est = Trajectory(traj.size());
    std::shared_ptr<ikf::LinearBelief> ptr_bel0(new ikf::LinearBelief());

    Eigen::VectorXd m_0(2,1);
    m_0 << traj.p_arr(0), traj.v_arr(0);

    auto gen = ikf::MultivariateNormal<double>(m_0.matrix(), Eigen::Matrix2d::Identity());

    auto init_mean = gen.samples(1);
    m_0 << init_mean(0,0), init_mean(1,0);
    ptr_bel0->mean(m_0);
    ptr_bel0->Sigma(Eigen::Matrix2d::Identity());
    ptr_bel0->set_timestamp(ikf::Timestamp(0.0));
    ptr_IKF->set_belief(ptr_bel0);

    p_noisy_arr = traj.generate_noisy_pos(std_dev_p);
    a_noisy_arr = traj.generate_noisy_acc(std_dev_a);


    traj_est.p_arr(0) = m_0(0);
    traj_est.v_arr(0) = m_0(1);
    traj_est.a_arr(0) = a_noisy_arr(0);
    traj_est.t_arr(0) = traj.t_arr(0);
  }

  void generate_rel_meas(Trajectory const& traj2, size_t const ID_2) {
    dict_p_rel_noisy_arr.emplace(ID_2, traj.generate_noisy_rel_pos(traj2, std_dev_p_rel));
  }

  bool propagate_idx(size_t const idx) {
    int dim_u = 1; // Number of inputs
    Eigen::VectorXd u (dim_u,1);  // control input
    Eigen::VectorXd u_var (dim_u,dim_u);  // control input covariance

    u_var << std_dev_a * std_dev_a;
    u << a_noisy_arr(idx);
    ikf::Timestamp t_curr(traj.t_arr(idx));
    if (ptr_IKF->predict(u, u_var)) {
      auto p_bel = ptr_IKF->get_belief();
      std::cout << "* Prop[" << ID <<  "]:t=" << p_bel->timestamp() << ", \nmean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
    }
    return true;
  }

  bool update_idx(size_t const idx) {
    int dim_x = 2; // Number of states
    int dim_z = 1; // Number of measurements
    Eigen::VectorXd z (dim_z,1);  // measurement

    ikf::Timestamp t_curr(traj.t_arr(idx));

    z << p_noisy_arr(idx);
    if (ptr_IKF->update(z)) {
      auto p_bel = ptr_IKF->get_belief();
      std::cout << "* Update [" << ID <<  "]:t=" << t_curr << ", mean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
    }

    for(auto & elem : dict_p_rel_noisy_arr) {
      size_t ID_J = elem.first;
      Eigen::MatrixXd R (dim_z,dim_z);  // measurement covariance

      Eigen::MatrixXd H_II(dim_z, dim_x); // Output matrix
      Eigen::MatrixXd H_JJ(dim_z, dim_x); // Output matrix
      R << std_dev_p_rel * std_dev_p_rel;
      H_II << -1, 0;
      H_JJ << 1, 0;

      z << elem.second(idx);
      if (ptr_IKF->joint_update(H_II, H_JJ, ID_J, R, z)) {
        auto p_bel = ptr_IKF->get_belief();
        std::cout << "* Update Rel [" << ID << "," << ID_J << "]:t=" << t_curr << ", mean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
      }

    }
    auto p_bel = ptr_IKF->get_belief();

    Eigen::VectorXd mean_apos = p_bel->mean();
    traj_est.p_arr(idx) = mean_apos(0);
    traj_est.v_arr(idx) = mean_apos(1);
    traj_est.a_arr(idx) = a_noisy_arr(idx);
    traj_est.t_arr(idx) = traj.t_arr(idx);

    return true;
  }

public:
  size_t ID = 0;
  double const dt;
  double const std_dev_p = 0.05;
  double const std_dev_a = 0.05;
  double const std_dev_p_rel = 0.05;
  std::shared_ptr<LinearIKF> ptr_IKF;
  Trajectory traj;
  Trajectory traj_est;
  Eigen::ArrayXd p_noisy_arr;
  Eigen::ArrayXd a_noisy_arr;
  std::map<size_t, Eigen::ArrayXd> dict_p_rel_noisy_arr;
};



#endif // SIMINSTANCE_HPP
