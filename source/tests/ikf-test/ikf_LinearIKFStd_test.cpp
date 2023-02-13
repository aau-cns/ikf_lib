/******************************************************************************
* FILENAME:     LinearIKF_test.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     03.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/EstimatorStd/LinearIKFStd.hpp>
#include <ikf/utils/eigen_utils.hpp>
#include <gmock/gmock.h>
#include <ikf/Estimator/LinearIKF.hpp>
#include <ikf/Estimator/IsolatedKalmanFilterHandler.hpp>
#include <ikf/utils/eigen_mvn.hpp>
#include <matplot/matplot.h>

class LinearIKF_test : public testing::Test
{
public:
};



TEST_F(LinearIKF_test, traj)
{


}
/*
TEST_F(LinearIKF_test, single_inst)
{
  std::shared_ptr<ikf::IsolatedKalmanFilterHandler> ptr_Handler(new ikf::IsolatedKalmanFilterHandler());

  // https://github.com/hmartiro/kalman-cpp/blob/master/kalman-test.cpp
  // https://www.kalmanfilter.net/modeling.html (Example continued: constant acceleration moving body)

  int dim_x = 2; // Number of states
  int dim_z = 1; // Number of measurements
  int dim_u = 1; // Number of inputs

  double const dt = 1.0/10; // Time step
  double const std_dev_p = 0.05;
  double const std_dev_a = 0.05;

  Eigen::MatrixXd F(dim_x, dim_x); // System dynamics matrix
  Eigen::MatrixXd G(dim_x, dim_u); // Control input matrix
  Eigen::MatrixXd Q(dim_x, dim_x); // Process noise covariance
  Eigen::MatrixXd H(dim_z, dim_x); // Output matrix
  Eigen::MatrixXd P0(dim_x, dim_x); // Estimate error covariance
  Eigen::MatrixXd R (dim_z,dim_z);  // measurement covariance
  Eigen::VectorXd z (dim_z,1);  // measurement
  Eigen::VectorXd u (dim_u,1);  // control input
  Eigen::VectorXd u_var (dim_u,dim_u);  // control input covariance

  // Discrete LTI projectile motion, measuring position only
  F << 1, dt, 0, 1;
  H << 1, 0;
  G << 0.5*dt*dt, dt;

  // Reasonable covariance matrices
  Q = Q.Identity(2,2)*0.01;
  P0 << Q;
  R << std_dev_p * std_dev_p;
  u_var << std_dev_a * std_dev_a;

  std::cout << "F: \n" << F << std::endl;
  std::cout << "H: \n" << H << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P0: \n" << P0 << std::endl;


  ikf::LinearIKFStd LIKF;
  LIKF.define_system(F, G, Q, H, R);

  Trajectory traj;
  traj.generate_sine(dt, 1, 0.1, 0, 1);

  std::shared_ptr<ikf::LinearBelief> ptr_bel0(new ikf::LinearBelief());

  Eigen::VectorXd m_0(2,1);
  m_0 << traj.p_arr(0), traj.v_arr(0);
  ptr_bel0->mean(m_0);
  ptr_bel0->Sigma(P0);
  LIKF.initialize(ptr_bel0, ikf::Timestamp(0.0));

  Eigen::ArrayXd p_noisy_arr = traj.generate_noisy_pos(std_dev_p);
  Eigen::ArrayXd a_noisy_arr = traj.generate_noisy_acc(std_dev_a);

  for (int i = 1; i < traj.t_arr.size(); i++) {
    ikf::Timestamp t_curr(traj.t_arr(i));
    u << a_noisy_arr(i);

    if (LIKF.propagate(dt, u, u_var)) {
      ikf::ptr_belief p_bel = LIKF.get_belief_at_t(LIKF.current_t());
      std::cout << "* Prop:t=" << LIKF.current_t() << ", \nmean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
    }

    z << p_noisy_arr(i);
    if (LIKF.update(z, t_curr)) {
      ikf::ptr_belief p_bel = LIKF.get_belief_at_t(t_curr);
      std::cout << "* Update:t=" << t_curr << ", mean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
    }
  }
}


class SimInstance {
public:
  SimInstance(const Eigen::MatrixXd &F, const Eigen::MatrixXd &G, const Eigen::MatrixXd &Q,
              const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
              size_t const ID, double const dt, double const D, double const omega,
              double const std_dev_p, double const std_dev_a, double const std_dev_p_rel,
              std::shared_ptr<ikf::IsolatedKalmanFilterHandler> ptr_Handler) : ID(ID), dt(dt), std_dev_p(std_dev_p), std_dev_a(std_dev_a), std_dev_p_rel(std_dev_p_rel), ptr_IKF(new ikf::LinearIKFStd(ptr_Handler, "name" + std::to_string(ID), ID)) {

    //double const omega = 0.4;
    double const omega_0 = 0.4;
    ptr_IKF->define_system(F, G, Q, H, R);
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
    ptr_IKF->initialize(ptr_bel0, ikf::Timestamp(0.0));

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
    if (ptr_IKF->propagate(dt, u, u_var)) {
      auto p_bel = ptr_IKF->get_belief_at_t(ptr_IKF->current_t());
      std::cout << "* Prop[" << ID <<  "]:t=" << ptr_IKF->current_t() << ", \nmean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
    }
    return true;
  }

  bool update_idx(size_t const idx) {
    int dim_x = 2; // Number of states
    int dim_z = 1; // Number of measurements
    Eigen::VectorXd z (dim_z,1);  // measurement

    ikf::Timestamp t_curr(traj.t_arr(idx));

    z << p_noisy_arr(idx);
    if (ptr_IKF->update(z, t_curr)) {
      auto p_bel = ptr_IKF->get_belief_at_t(t_curr);
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
      if (ptr_IKF->update_joint(H_II, H_JJ, ID_J, R, z, t_curr)) {
        auto p_bel = ptr_IKF->get_belief_at_t(t_curr);
        std::cout << "* Update Rel [" << ID << "," << ID_J << "]:t=" << t_curr << ", mean=\n " << p_bel->mean() << ",\nSigma=\n" << p_bel->Sigma() << std::endl;
      }

    }
    auto p_bel = ptr_IKF->get_belief_at_t(t_curr);

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
  std::shared_ptr<ikf::LinearIKFStd> ptr_IKF;
  Trajectory traj;
  Trajectory traj_est;
  Eigen::ArrayXd p_noisy_arr;
  Eigen::ArrayXd a_noisy_arr;
  std::map<size_t, Eigen::ArrayXd> dict_p_rel_noisy_arr;
};

TEST_F(LinearIKF_test, multi_inst)
{
  std::shared_ptr<ikf::IsolatedKalmanFilterHandler> ptr_Handler(new ikf::IsolatedKalmanFilterHandler());

  // https://github.com/hmartiro/kalman-cpp/blob/master/kalman-test.cpp
  // https://www.kalmanfilter.net/modeling.html (Example continued: constant acceleration moving body)

  int dim_x = 2; // Number of states
  int dim_z = 1; // Number of measurements
  int dim_u = 1; // Number of inputs

  double const dt = 1.0/100; // Time step
  double const D = 2;
  double const omega = M_PI;
  double const std_dev_p = 0.05;
  double const std_dev_a = 0.05;
  double const std_dev_p_rel = 0.05;

  Eigen::MatrixXd F(dim_x, dim_x); // System dynamics matrix
  Eigen::MatrixXd G(dim_x, dim_u); // Control input matrix
  Eigen::MatrixXd Q(dim_x, dim_x); // Process noise covariance
  Eigen::MatrixXd H(dim_z, dim_x); // Output matrix
  Eigen::MatrixXd P0(dim_x, dim_x); // Estimate error covariance
  Eigen::MatrixXd R (dim_z,dim_z);  // measurement covariance
  Eigen::VectorXd z (dim_z,1);  // measurement
  Eigen::VectorXd u (dim_u,1);  // control input
  Eigen::VectorXd u_var (dim_u,dim_u);  // control input covariance

  // Discrete LTI projectile motion, measuring position only
  F << 1, dt, 0, 1;
  H << 1, 0;
  G << 0.5*dt*dt, dt;

  // Reasonable covariance matrices
  Q = Q.Identity(2,2)*0.01;
  P0 << Q;
  R << std_dev_p * std_dev_p;
  u_var << std_dev_a * std_dev_a;

  std::cout << "F: \n" << F << std::endl;
  std::cout << "H: \n" << H << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P0: \n" << P0 << std::endl;



  SimInstance inst1(F, G, Q, H, R, 1, dt, D, omega, std_dev_p, std_dev_a, std_dev_p_rel, ptr_Handler);
  SimInstance inst2(F, G, Q, H, R, 2, dt, D, omega, std_dev_p, std_dev_a, std_dev_p_rel, ptr_Handler);
  ptr_Handler->add(inst1.ptr_IKF);
  ptr_Handler->add(inst2.ptr_IKF);

  inst1.ptr_IKF->plot_estimate();

  inst1.generate_rel_meas(inst2.traj, inst2.ID);


  for (int i = 1; i < inst1.traj.t_arr.size(); i++) {
    inst1.propagate_idx(i);
    inst2.propagate_idx(i);
    inst1.update_idx(i);
    inst2.update_idx(i);
  }

  inst1.traj.plot_trajectory(0);
  inst1.traj_est.plot_trajectory(1);



}
*/
