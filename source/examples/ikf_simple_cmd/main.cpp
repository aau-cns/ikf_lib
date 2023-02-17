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
  int const N = 4; // number of filter instances
  int const num_instances = std::max(N, 1); // at least 1 is needed!

  std::cout << "ikf_cmd example: 1D constant acceleration moving body (harmonic motion)" << std::endl;
  std::cout << "* \t p = sin(t*omega   + omega_0)*amplitude + offset" << std::endl;
  std::cout << "* \t v = omega* cost(t*omega  + omega_0)*amplitude" << std::endl;
  std::cout << "* \t a = -omega*omega*sin(omega*t_arr  + omega_0)*amplitude" << std::endl;
  std::cout << "* \t omega_0 = PI/8*ID" << std::endl;
  std::cout << "* \t amplitude = 1+0.1*ID" << std::endl;
  std::cout << "* \t offset = 0" << std::endl;
  std::cout << "* \t ID = number of filter instance [0,N-1]" << std::endl;
  std::cout << "* \t N = " << num_instances << std::endl;
  std::cout << "* \t Sigma(0) = eye(2)*0.5 " << num_instances << std::endl;
  std::cout << "* \t mean(0) = randn(Sigma(0)) " << num_instances << std::endl;
  std::cout << "noisy control input 'a' with std_dev_a for prediction" << std::endl;
  std::cout << "noisy position measurement 'p' with std_dev_p for private state correction" << std::endl;
  std::cout << " -> noisy position measurement is just obtained by filter instance 0" << std::endl;
  std::cout << "noisy relative position measurement 'p_i_j' with std_dev_p_rel for isolated joint state correction between filter i and j using the Isolated Kalman Filter." << std::endl;
  std::cout << "for system model see: https://www.kalmanfilter.net/modeling.html (Example continued: constant acceleration moving body)" << std::endl;

  ikf::GaussianNoiseGen& gen = ikf::GaussianNoiseGen::instance();
  gen.seed(123123);
  std::shared_ptr<ikf::IKFHandlerStd> ptr_Handler(new ikf::IKFHandlerStd());
  int const dim_x = 2; // Number of states
  int const dim_z = 1; // Number of measurements
  int const dim_u = 1; // Number of inputs

  double const dt = 1.0/100; // Time step
  double const D = 5;
  double const omega = M_PI/2;
  double const std_dev_p = 0.05;
  double const std_dev_a = 0.05;
  double const std_dev_p_rel = 0.05;

  Eigen::MatrixXd F(dim_x, dim_x); // System dynamics matrix
  Eigen::MatrixXd G(dim_x, dim_u); // Control input matrix
  Eigen::MatrixXd Q(dim_x, dim_x); // Process noise covariance
  Eigen::MatrixXd H_private(dim_z, dim_x); // Output matrix
  Eigen::MatrixXd H_joint(dim_z, dim_x*2); // Output matrix
  Eigen::MatrixXd R_private (dim_z,dim_z);  // measurement covariance
  Eigen::MatrixXd R_joint (dim_z,dim_z);  // measurement covariance
  Eigen::VectorXd z (dim_z,1);  // measurement
  Eigen::VectorXd u (dim_u,1);  // control input
  Eigen::VectorXd u_var (dim_u,dim_u);  // control input covariance

  // Discrete LTI projectile motion, measuring position only
  F << 1, dt, 0, 1;
  H_private << 1, 0;
  H_joint << -1, 0, 1, 0;
  G << 0.5*dt*dt, dt;

  // Reasonable covariance matrices
  Q = Q.Identity(2,2)*0.000001;
  R_private << std_dev_p * std_dev_p;
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
  std::cout << "H_private (Measurement Sensitivity Matrix): \n" << H_private << std::endl;
  std::cout << "R_private (Measurement covariance): \n" << R_private << std::endl;
  std::cout << "H_joint (Measurement Sensitivity Matrix): \n" << H_joint << std::endl;
  std::cout << "R_joint (Measurement covariance): \n" << R_joint << std::endl;

  typedef std::shared_ptr<SimInstance> ptr_Instance;
  std::unordered_map<int, ptr_Instance> dict_instance;

  for(int i=0; i < num_instances; i++) {
    size_t ID = i;
    dict_instance[i] = ptr_Instance(new SimInstance(F, G, Q, H_private, R_private, ID, dt, D, omega, std_dev_p, std_dev_a, std_dev_p_rel, ptr_Handler));
    ptr_Handler->add(dict_instance[i]->ptr_IKF);
    if (i > 0) {
      dict_instance[i]->perform_private = false;
    }
  }

  if (num_instances > 1) {
    for(int i=0; i < num_instances; i++) {
      size_t ID_I = i;
      size_t ID_J = (i + 1) % num_instances;
      dict_instance[ID_I]->generate_rel_meas(dict_instance[ID_J]->traj, dict_instance[ID_J]->ID);
    }
  }



  for (int t = 1; t < dict_instance[0]->traj.t_arr.size(); t++) {
    for(int i=0; i < num_instances; i++) {
      size_t ID_I = i;
      dict_instance[ID_I]->propagate_idx(t);
    }
    for(int i=0; i < num_instances; i++) {
      size_t ID_I = i;
      dict_instance[ID_I]->update_idx(t);
    }
  }

  for(int i=0; i < num_instances; i++) {
    std::cout << "Beliefs of filter instance ID=" << i << std::endl;
    dict_instance[i]->print_HistBelief(20);
  }

  for(int i=0; i < num_instances; i++) {
    dict_instance[i]->traj.plot_trajectory(i, "True");
    dict_instance[i]->traj_est.plot_trajectory(i, "Est");
    dict_instance[i]->compute_error();
    dict_instance[i]->traj_err.plot_trajectory(i, "Err");
  }

  wait_for_key();


  return 0;
}
