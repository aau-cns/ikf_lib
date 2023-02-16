#include "include/Trajectory.hpp"
#include "include/SimInstanceDelay.hpp"
#include <ikf/Estimator/IsolatedKalmanFilterHandler.hpp>



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
  int num_instances = 2;
  // Calculate and print fibonacci number
  std::cout << "ikf_delay_cmd example: 1D constant acceleration moving body (harmonic motion)" << std::endl;
  std::cout << "* \t p = sin(t*omega   + omega_0)*amplitude + offset" << std::endl;
  std::cout << "* \t v = omega* cost(t*omega  + omega_0)*amplitude" << std::endl;
  std::cout << "* \t a = -omega*omega*sin(omega*t_arr  + omega_0)*amplitude" << std::endl;
  std::cout << "* \t omega_0 = 0.4*ID" << std::endl;
  std::cout << "* \t amplitude = 1+0.1*ID" << std::endl;
  std::cout << "* \t offset = ID" << std::endl;
  std::cout << "* \t ID = number of filter instance [0,N-1]" << std::endl;
  std::cout << "* \t N = " << num_instances << std::endl;
  std::cout << "noisy control input 'a' with std_dev_a for prediction" << std::endl;
  std::cout << "noisy position measurement 'p' with std_dev_p for private state correction" << std::endl;
  std::cout << "noisy relative position measurement 'p_i_j' with std_dev_p_rel for isolated joint state correction between filter 1 and 2 using the Isolated Kalman Filter." << std::endl;
  std::cout << "see: https://www.kalmanfilter.net/modeling.html (Example continued: constant acceleration moving body)" << std::endl;


  std::shared_ptr<ikf::IsolatedKalmanFilterHandler> ptr_Handler(new ikf::IsolatedKalmanFilterHandler());

  // https://github.com/hmartiro/kalman-cpp/blob/master/kalman-test.cpp
  // https://www.kalmanfilter.net/modeling.html (Example continued: constant acceleration moving body)

  ikf::GaussianNoiseGen& gen = ikf::GaussianNoiseGen::instance();
  gen.seed(123123);
  double const dt = 1.0/100; // Time step
  double const D = 5;
  double const omega = M_PI/2;
  double const std_dev_p = 0.05;
  double const std_dev_a = 0.05;
  double const std_dev_p_rel = 0.05;


  std::cout << "Duration D: " << D << std::endl;
  std::cout << "dt: " << dt << std::endl;
  std::cout << "std_dev_p: " << std_dev_p << std::endl;
  std::cout << "std_dev_a: " << std_dev_a << std::endl;
  std::cout << "std_dev_p_rel: " << std_dev_a << std::endl;
  std::cout << "omega: " << std_dev_a << std::endl;


  typedef std::shared_ptr<SimInstanceDelay> ptr_Instance;
  std::unordered_map<int, ptr_Instance> dict_instance;

  for(int i=0; i < num_instances; i++) {
    size_t ID = i;
    dict_instance[i] = ptr_Instance(new SimInstanceDelay(ID, dt, D, omega, std_dev_p, std_dev_a, std_dev_p_rel, ptr_Handler));
    ptr_Handler->add(dict_instance[i]->ptr_IKF);
    if (i > 0) {
      dict_instance[i]->perform_private = false;
    }
  }


  for(int i=0; i < num_instances; i++) {
    size_t ID_I = i;
    size_t ID_J = (i + 1) % num_instances;
    dict_instance[ID_I]->generate_rel_meas(dict_instance[ID_J]->traj, dict_instance[ID_J]->ID);
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
    std::cout << "MeasData of filter instance ID=" << dict_instance[i]->ptr_IKF->ID() << std::endl;
    dict_instance[i]->ptr_IKF->print_HistMeas(20);
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
