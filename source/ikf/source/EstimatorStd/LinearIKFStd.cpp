/******************************************************************************
* FILENAME:     SimpleLinearIKFStd.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     06.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/utils/eigen_utils.hpp>
#include <ikf/EstimatorStd/LinearIKFStd.hpp>
#include <ikf/Estimator/IsolatedKalmanFilterHandler.hpp>
#include <matplot/matplot.h>


namespace ikf {

LinearIKFStd::LinearIKFStd(const bool handle_delayed_meas, const double horizon_sec) : IIsolatedKalmanFilter(std::shared_ptr<IsolatedKalmanFilterHandler>(nullptr), "unknown", 1, handle_delayed_meas, horizon_sec) {}

LinearIKFStd::LinearIKFStd(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, const std::string &name, const size_t ID, const bool handle_delayed_meas, const double horizon_sec): IIsolatedKalmanFilter(ptr_Handler, name, ID, handle_delayed_meas, horizon_sec) {}

LinearIKFStd::~LinearIKFStd() {}

void LinearIKFStd::define_system(const Eigen::MatrixXd &F, const Eigen::MatrixXd &G, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
  m_F = F; m_G = G; m_Q = Q; m_H_priv = H, m_R=R;
  m_defined = true;
}

bool LinearIKFStd::propagate(const double dt) {
  if (m_defined) {
    return propagate(m_F, m_Q, dt);
  }
  std::cout << "System not defined!" << std::endl;
  return false;
}

bool LinearIKFStd::propagate(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, const double dt) {
  Timestamp t_a = current_t();
  RTV_EXPECT_TRUE_MSG(dt > 0.0, "Measurement must be ahead of the current state at t_a" + t_a.str());
  return apply_propagation(F, Q, t_a, Timestamp(t_a.to_sec() + dt));
}

bool LinearIKFStd::propagate(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, const double dt, const Eigen::MatrixXd &G, const Eigen::VectorXd &u, const Eigen::VectorXd &var_u) {
  Timestamp t_a = current_t();
  ptr_belief p_bel_a = get_belief_at_t(t_a);
  RTV_EXPECT_TRUE_MSG(dt > 0.0, "Measurement must be ahead of the current state at t_a" + t_a.str());

  Eigen::VectorXd mean_b = (F * p_bel_a->mean() + G * u);
  Eigen::MatrixXd Q_d = G * var_u * G.transpose() + Q;

  return apply_propagation(p_bel_a, mean_b, F, Q_d, t_a, Timestamp(t_a.to_sec() + dt));
}

bool LinearIKFStd::propagate(const double dt, const Eigen::VectorXd &u, const Eigen::VectorXd &var_u) {
  if (m_defined) {
    return propagate(m_F, m_Q, dt, m_G, u, var_u);
  }
  std::cout << "System not defined!" << std::endl;
  return false;
}

bool LinearIKFStd::update(const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t) {

  return apply_private_observation(H, R, z, t);
}

bool LinearIKFStd::update(const Eigen::VectorXd &z, const Timestamp &t) {
  if (m_defined) {
    return update(m_H_priv, m_R, z, t);
  }
  std::cout << "System not defined!" << std::endl;
  return false;
}

bool LinearIKFStd::update_joint(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const size_t ID_J, const Eigen::MatrixXd &R, const Eigen::VectorXd &z, const Timestamp &t) {
    return apply_joint_observation(m_ID, ID_J, H_II, H_JJ, R, z, t);
}

bool LinearIKFStd::plot_estimate() {
    using namespace matplot;
    auto f1 = figure();

    std::vector<double> x = linspace(0, 10);
    std::vector<double> y = transform(x, [](auto x) { return sin(x); });
    plot(x, y, "-o")->marker_indices({0,  5,  10, 15, 20, 25, 30, 35, 40, 45,
                                      50, 55, 60, 65, 70, 75, 80, 85, 90, 95});

    f1->draw();
    return true;
}

ProcessMeasResult_t LinearIKFStd::progapation_measurement(const MeasData &m) {
  RTV_EXPECT_TRUE_THROW(false, "MeasData not supported!");
  return ProcessMeasResult_t();
}

ProcessMeasResult_t LinearIKFStd::local_private_measurement(const MeasData &m) {
  RTV_EXPECT_TRUE_THROW(false, "MeasData not supported!");
  return ProcessMeasResult_t();
}

ProcessMeasResult_t LinearIKFStd::local_joint_measurement(const MeasData &m) {
  RTV_EXPECT_TRUE_THROW(false, "MeasData not supported!");
  return ProcessMeasResult_t();
}


} // ns mmsf
