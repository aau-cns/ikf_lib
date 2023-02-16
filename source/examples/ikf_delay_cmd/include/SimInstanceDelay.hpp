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
#include "LinearIKF_1D_const_acc.hpp"
#include "ikf/Estimator/IsolatedKalmanFilterHandler.hpp"
#include <ikf/Estimate/LinearBelief.hpp>



class SimInstanceDelay {
public:
  SimInstanceDelay(size_t const ID, double const dt, double const D, double const omega,
              double const std_dev_p, double const std_dev_a, double const std_dev_p_rel,
              std::shared_ptr<ikf::IsolatedKalmanFilterHandler> ptr_Handler) : ID(ID), dt(dt), std_dev_p(std_dev_p), std_dev_a(std_dev_a), std_dev_p_rel(std_dev_p_rel), ptr_IKF(new LinearIKF_1D_const_acc(ptr_Handler, ID)), HistBelief(1.0) {

    double const omega_0 = M_PI/8;
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
    ptr_IKF->initialize(ptr_bel0, ptr_bel0->timestamp());

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
    ikf::Timestamp t_curr(traj.t_arr(idx));
    ikf::MeasData m;
    m.obs_type = ikf::eObservationType::PROPAGATION;
    m.meas_type = "acceleration";
    m.z.setZero(1,1);
    m.R.setZero(1,1);
    m.z << a_noisy_arr(idx);
    m.R << std_dev_a * std_dev_a;
    m.t_m = t_curr;
    m.t_p = t_curr;

    ikf::ProcessMeasResult_t res = ptr_IKF->process_measurement(m);
    if (print_belief) {
      auto p_bel = ptr_IKF->current_belief();
      std::cout << "* Prop[" << ID <<  "]:t=" << p_bel->timestamp() << ", \nmean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
    }
    return true;
  }

  bool update_idx(size_t const idx) {
    ikf::Timestamp t_curr(traj.t_arr(idx));
    if (perform_private && idx > delay_private)
    {
      size_t const idx_meas = idx - delay_private;
      ikf::Timestamp t_meas(traj.t_arr(idx_meas));
      ikf::MeasData m;
      m.obs_type = ikf::eObservationType::PRIVATE_OBSERVATION;
      m.meas_type = "position";
      m.z.setZero(1,1);
      m.R.setZero(1,1);
      m.z << p_noisy_arr(idx_meas);
      m.R << std_dev_p * std_dev_p;
      m.t_m = t_meas;
      m.t_p = t_curr;

      ikf::ProcessMeasResult_t res = ptr_IKF->process_measurement(m);
      if (print_belief) {
        auto p_bel = ptr_IKF->current_belief();
        std::cout << "* Update [" << ID <<  "]:t=" << t_curr << ", mean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
      }

      // Apply correct beliefs from the past!
      if (delay_private) {
        for (int i = delay_private; i > 0; i--) {
          size_t const idx_meas = idx - i;
          ikf::Timestamp t_meas(traj.t_arr(idx_meas));
          auto p_bel = ptr_IKF->get_belief_at_t(t_meas);
          Eigen::VectorXd mean_apos = p_bel->mean();
          traj_est.p_arr(idx_meas) = mean_apos(0);
          traj_est.v_arr(idx_meas) = mean_apos(1);
        }
      }

    }

    if (perform_joint && idx > delay_joint)
    {
      for(auto & elem : dict_p_rel_noisy_arr) {
        size_t const idx_meas = idx - delay_joint;
        ikf::Timestamp t_meas(traj.t_arr(idx_meas));

        size_t ID_J = elem.first;
        ikf::MeasData m;
        m.obs_type = ikf::eObservationType::JOINT_OBSERVATION;
        m.meas_type = "relative_position";
        m.meta_info = std::to_string(ID_J);

        m.z.setZero(1,1);
        m.R.setZero(1,1);
        m.z << elem.second(idx_meas);
        m.R << std_dev_p_rel * std_dev_p_rel;
        m.t_m = t_meas;
        m.t_p = t_curr;

        ikf::ProcessMeasResult_t res = ptr_IKF->process_measurement(m);
        if (print_belief) {
          auto p_bel = ptr_IKF->current_belief();
          std::cout << "* Update Rel [" << ID << "," << ID_J << "]:t=" << t_curr << ", mean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
        }

        // Apply correct beliefs from the past!
        if (delay_joint) {
          for (int i = delay_joint; i > 0; i--) {
            size_t const idx_meas = idx - i;
            ikf::Timestamp t_meas(traj.t_arr(idx_meas));
            auto p_bel = ptr_IKF->get_belief_at_t(t_meas);
            Eigen::VectorXd mean_apos = p_bel->mean();
            traj_est.p_arr(idx_meas) = mean_apos(0);
            traj_est.v_arr(idx_meas) = mean_apos(1);
          }
        }

      }

    }

    auto p_bel = ptr_IKF->current_belief();
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
    HistBelief.foreach([&cnt, max](ikf::ptr_belief const& i){
      if(cnt < max) {
        std::cout << (*i.get()) << std::endl;
      }
      cnt++;
    });
  }


public:
  size_t ID = 0;
  size_t delay_private = 0;
  size_t delay_joint = 0;
  bool perform_private = true;
  bool perform_joint = true;
  bool print_belief = false;
  double const dt;
  double const std_dev_p = 0.05;
  double const std_dev_a = 0.05;
  double const std_dev_p_rel = 0.05;
  std::shared_ptr<LinearIKF_1D_const_acc> ptr_IKF;
  Trajectory traj;
  Trajectory traj_est;
  Trajectory traj_err;
  Eigen::ArrayXd p_noisy_arr;
  Eigen::ArrayXd a_noisy_arr;
  std::map<size_t, Eigen::ArrayXd> dict_p_rel_noisy_arr;
  ikf::TTimeHorizonBuffer<ikf::ptr_belief> HistBelief;
};



#endif // SIMINSTANCE_HPP
