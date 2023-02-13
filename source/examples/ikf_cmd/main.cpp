#include "Trajectory.hpp"
#include "SimInstance.hpp"
#include <ikf/EstimatorStd/IKFHandlerStd.hpp>

void multi_inst()
{
  std::shared_ptr<ikf::IKFHandlerStd> ptr_Handler(new ikf::IKFHandlerStd());

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


  inst1.generate_rel_meas(inst2.traj, inst2.ID);


  for (int i = 1; i < inst1.traj.t_arr.size(); i++) {
    inst1.propagate_idx(i);
    inst2.propagate_idx(i);
    inst1.update_idx(i);
    inst2.update_idx(i);
  }

  inst1.traj.plot_trajectory(0);
  inst1.traj_est.plot_trajectory(1);

  std::cout << "press a value to continue..." << std::flush << std::endl;

  int Value;
  for(;;)
  {
    if(std::cin >> Value)
      break;
    std::cin.clear();
    std::cin.ignore(INT_MAX,'\n');
  }
  std::cout << "The value has been set to: " << Value << std::endl;
}


int main(int /*argc*/, char** /*argv[]*/)
{
  // Calculate and print fibonacci number
  std::cout << "ikf_cmd example" << std::endl;

  Trajectory traj;
  traj.generate_sine(0.1, 1, 0.1, 0, 1);
  traj.print();
  traj.plot_trajectory();

  multi_inst();



  return 0;
}
