/******************************************************************************
* FILENAME:     SimInstance.hpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
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
#ifndef SIMINSTANCE_HPP
#define SIMINSTANCE_HPP
#include "Trajectory.hpp"
#include "LinearIKF.hpp"
#include "ikf/Container/TTimeHorizonBuffer.hpp"
#include <ikf/Estimate/LinearBelief.hpp>
#include <ikf/utils/MultivariateNormal.hpp>


class SimInstance {
public:
  SimInstance(const Eigen::MatrixXd &F, const Eigen::MatrixXd &G, const Eigen::MatrixXd &Q,
              const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
              size_t const ID, double const dt, double const D, double const omega,
              double const std_dev_p, double const std_dev_a, double const std_dev_p_rel,
              std::shared_ptr<ikf::IKFHandlerStd> ptr_Handler) : ID(ID), dt(dt), std_dev_p(std_dev_p), std_dev_a(std_dev_a), std_dev_p_rel(std_dev_p_rel), ptr_IKF(new LinearIKF(ptr_Handler, ID)), HistBelief(1.0) {

    double const omega_0 = M_PI/8;
    ptr_IKF->define_system(F, G, Q, H, R, dt);
    traj.generate_sine(dt, D, omega, omega_0*ID, (0.1*ID+1), 0);

    traj_est = Trajectory(traj.size());
    traj_err = Trajectory(traj.size());
    std::shared_ptr<ikf::LinearBelief> ptr_bel0(new ikf::LinearBelief());

    Eigen::VectorXd m_0(2,1);
    m_0 << traj.p_arr(0), traj.v_arr(0);


    auto Sigma_0 = Eigen::Matrix2d::Identity()*0.5;

    auto gen = ikf::MultivariateNormal<double>(m_0.matrix(), Sigma_0);
    auto init_mean = gen.samples(1);
    m_0 << init_mean(0,0), init_mean(1,0);
    ptr_bel0->mean(m_0);
    ptr_bel0->Sigma(Sigma_0);
    ptr_bel0->set_timestamp(ikf::Timestamp(0.0));
    ptr_IKF->set_belief(ptr_bel0);

    p_noisy_arr = traj.generate_noisy_pos(std_dev_p);
    a_noisy_arr = traj.generate_noisy_acc(std_dev_a);


    traj_est.p_arr(0) = m_0(0);
    traj_est.v_arr(0) = m_0(1);
    traj_est.a_arr(0) = a_noisy_arr(0);
    traj_est.t_arr(0) = traj.t_arr(0);

    HistBelief.insert(ptr_bel0->clone(), ptr_bel0->timestamp());
  }

  void compute_error() {
    double  rmse_p = 0, rmse_v = 0;

    for (int t = 0; t < traj.t_arr.size(); t++)
    {
      traj_err.p_arr(t) =  traj.p_arr(t) - traj_est.p_arr(t);
      traj_err.v_arr(t) =  traj.v_arr(t) - traj_est.v_arr(t);
      traj_err.a_arr(t) =  traj.a_arr(t) - traj_est.a_arr(t);
      traj_err.t_arr(t) = traj.t_arr(t);

      rmse_p += traj_err.p_arr(t)*traj_err.p_arr(t);
      rmse_v += traj_err.v_arr(t)*traj_err.v_arr(t);
    }
    rmse_p /= traj.t_arr.size();
    rmse_v /= traj.t_arr.size();

    rmse_p = std::sqrt(rmse_p);
    rmse_v = std::sqrt(rmse_v);
    std::cout << "* RMSE[" << ID <<  "]: p=" << rmse_p << ", v= " << rmse_v << std::endl;
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
    if (ptr_IKF->predict(u, u_var)  && print_belief) {
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

    if (perform_private)
    {
      z << p_noisy_arr(idx);
      if (ptr_IKF->update(z)  && print_belief) {
        auto p_bel = ptr_IKF->get_belief();
        std::cout << "* Update [" << ID <<  "]:t=" << t_curr << ", mean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
      }
    }

    if (perform_joint)
    {
      for(auto & elem : dict_p_rel_noisy_arr) {
        size_t ID_J = elem.first;
        Eigen::MatrixXd R (dim_z,dim_z);  // measurement covariance

        Eigen::MatrixXd H_II(dim_z, dim_x); // Output matrix
        Eigen::MatrixXd H_JJ(dim_z, dim_x); // Output matrix
        R << std_dev_p_rel * std_dev_p_rel;
        H_II << -1, 0;
        H_JJ << 1, 0;

        z << elem.second(idx);
        if (ptr_IKF->joint_update(H_II, H_JJ, ID_J, R, z) && print_belief) {
          auto p_bel = ptr_IKF->get_belief();
          std::cout << "* Update Rel [" << ID << "," << ID_J << "]:t=" << t_curr << ", mean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
        }

      }
    }

    auto p_bel = ptr_IKF->get_belief()->clone();
    p_bel->set_timestamp(ikf::Timestamp(traj.t_arr(idx)));
    Eigen::VectorXd mean_apos = p_bel->mean();
    traj_est.p_arr(idx) = mean_apos(0);
    traj_est.v_arr(idx) = mean_apos(1);
    traj_est.a_arr(idx) = a_noisy_arr(idx);
    traj_est.t_arr(idx) = traj.t_arr(idx);
    HistBelief.insert(p_bel->clone(), p_bel->timestamp());

    return true;
  }

  void print_HistBelief(size_t max) {
    size_t cnt = 0;
    HistBelief.foreach([&cnt, max](ikf::pBelief_t const& i){
      if(cnt < max) {
        std::cout << (*i.get()) << std::endl;
      }
      cnt++;
    });
  }


public:
  size_t ID = 0;
  bool perform_private = true;
  bool perform_joint = true;
  bool print_belief = false;
  double const dt;
  double const std_dev_p = 0.05;
  double const std_dev_a = 0.05;
  double const std_dev_p_rel = 0.05;
  std::shared_ptr<LinearIKF> ptr_IKF;
  Trajectory traj;
  Trajectory traj_est;
  Trajectory traj_err;
  Eigen::ArrayXd p_noisy_arr;
  Eigen::ArrayXd a_noisy_arr;
  std::map<size_t, Eigen::ArrayXd> dict_p_rel_noisy_arr;
  ikf::TTimeHorizonBuffer<ikf::pBelief_t> HistBelief;
};



#endif // SIMINSTANCE_HPP
