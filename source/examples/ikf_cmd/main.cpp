#include "Trajectory.hpp"
#include "SimInstance.hpp"
#include <ikf/EstimatorStd/IKFHandlerStd.hpp>



void wait_for_key() {
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
  std::cout << "ikf_cmd example: 1D constant acceleration moving body (harmonic motion)" << std::endl;
  std::cout << "* \t p = sin(t*omega   + omega_0)*amplitude + offset" << std::endl;
  std::cout << "* \t v = omega* cost(t*omega  + omega_0)*amplitude" << std::endl;
  std::cout << "* \t a = -omega*omega*sin(omega*t_arr  + omega_0)*amplitude" << std::endl;
  std::cout << "* \t omega_0 = 0.4*ID" << std::endl;
  std::cout << "* \t amplitude = ID" << std::endl;
  std::cout << "* \t offset = ID" << std::endl;
  std::cout << "* \t ID = number of filter instance [1,2]" << std::endl;
  std::cout << "noisy control input 'a' with std_dev_a for prediction" << std::endl;
  std::cout << "noisy position measurement 'p' with std_dev_p for private state correction" << std::endl;
  std::cout << "noisy relative position measurement 'p_1_2' with std_dev_p_rel for isolated joint state correction between filter 1 and 2 using the Isolated Kalman Filter." << std::endl;
  std::cout << "see: https://www.kalmanfilter.net/modeling.html (Example continued: constant acceleration moving body)" << std::endl;


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
  Eigen::MatrixXd H_joint(dim_z, dim_x*2); // Output matrix
  Eigen::MatrixXd R (dim_z,dim_z);  // measurement covariance
  Eigen::MatrixXd R_joint (dim_z,dim_z);  // measurement covariance
  Eigen::VectorXd z (dim_z,1);  // measurement
  Eigen::VectorXd u (dim_u,1);  // control input
  Eigen::VectorXd u_var (dim_u,dim_u);  // control input covariance

  // Discrete LTI projectile motion, measuring position only
  F << 1, dt, 0, 1;
  H << 1, 0;
  H_joint << -1, 0, 1, 0;
  G << 0.5*dt*dt, dt;

  // Reasonable covariance matrices
  Q = Q.Identity(2,2)*0.01;
  R << std_dev_p * std_dev_p;
  R_joint << std_dev_p_rel * std_dev_p_rel;
  u_var << std_dev_a * std_dev_a;

  std::cout << "Duration D: " << D << std::endl;
  std::cout << "dt: " << dt << std::endl;
  std::cout << "std_dev_p: " << std_dev_p << std::endl;
  std::cout << "std_dev_a: " << std_dev_a << std::endl;
  std::cout << "std_dev_p_rel: " << std_dev_a << std::endl;
  std::cout << "u_var: " << u_var << std::endl;
  std::cout << "omega: " << std_dev_a << std::endl;

  std::cout << "F (System dynamics matrix): \n" << F << std::endl;
  std::cout << "Q (Process noise covariance): \n" << Q << std::endl;
  std::cout << "G (Control input matrix): \n" << G << std::endl;
  std::cout << "H_private (Measurement Sensitivity Matrix): \n" << H << std::endl;
  std::cout << "R_private (Measurement covariance): \n" << R << std::endl;
  std::cout << "H_joint (Measurement Sensitivity Matrix): \n" << H_joint << std::endl;
  std::cout << "R_joint (Measurement covariance): \n" << R_joint << std::endl;







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

  wait_for_key();


  return 0;
}
