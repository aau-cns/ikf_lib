#include "include/CLI11.hpp"
#include "include/SimInstanceDelay.hpp"
#include "include/Trajectory.hpp"
#include <ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp>

void wait_for_key() {
  std::cout << "insert a VALUE to continue..." << std::flush << std::endl;
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

int main(int argc, char** argv) {
  std::string app_name = "ikf_delay_cmd";
  CLI::App app{app_name};

  bool handle_measurements_centralized = true;

  int N = 4;  // number of filter instances
  app.add_option("--num_instances", N, "number of filter instances", true);

  int duration = 5;
  app.add_option("--duration", duration, "Duration of the trajectory [sec]", true);

  bool first_private_only = true;
  app.add_option("--first_private_only", first_private_only, "specifies if the first instance is obtaining private observations only", true);

  bool joint_updates = true;
  app.add_option("--joint_updates", joint_updates, "specifies if cyclic joint observations are performed (ID_J = (ID_I + 1) % num_instances)", true);


  int delay_private = 1;
  app.add_option("--delay_private", delay_private, "number of steps private measurements are delayed ", true);

  int delay_joint = 1;
  app.add_option("--delay_joint", delay_joint, "number of steps joint measurements are delayed ", true);

  bool list_beliefs = false;
  app.add_option("--list_beliefs", list_beliefs, "show a list of bliefs", true);

  bool show_plots = true; // number of filter instances
  app.add_option("--show_plots", show_plots, "show plots of the estimated trajectories", true);

  int seed = 123123;
  app.add_option("--seed", seed, "seed of random number generator", true);

  int freq = 100;
  app.add_option("--frequency", freq, "frequency of propagations", true);

  double omega = M_PI/2;
  app.add_option("--omega", omega, "omega, angular frequency of harominc", true);

  double std_dev_p = 0.05;
  app.add_option("--std_dev_p", std_dev_p, "position measurement noise", true);

  double std_dev_a = 0.05;
  app.add_option("--std_dev_a", std_dev_a, "acceleration input noise", true);

  double std_dev_p_rel = 0.05;
  app.add_option("--std_dev_p_rel", std_dev_p_rel, "relative position measurement noise", true);


  CLI11_PARSE(app, argc, argv);

  int const num_instances = std::max(N, 1); // at least 1 is needed!
  std::cout << "ikf_delay_cmd example: 1D constant acceleration moving body (harmonic motion)" << std::endl;
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

   std::cout << "* \t first_private_only = " << first_private_only << std::endl;
   std::cout << "* \t delay_private = " << delay_private << std::endl;
   std::cout << "* \t delay_joint = " << delay_joint << std::endl;
  if (handle_measurements_centralized) {
    std::cout << "* Measurements are maintained in IKF handler (centralized)"  << std::endl;
  }

  ikf::GaussianNoiseGen& gen = ikf::GaussianNoiseGen::instance();
  gen.seed(seed);

  std::shared_ptr<ikf::IsolatedKalmanFilterHandler> ptr_Handler(new ikf::IsolatedKalmanFilterHandler());

  double const dt = 1.0 / std::max(1, freq);  // Time step
  double const D = duration * 1.0;

  // we need the entire horizon to obtain all estimates after the simulation
  ptr_Handler->set_horizon(D);

  std::cout << "* Duration D: " << D << std::endl;
  std::cout << "* dt: " << dt << std::endl;
  std::cout << "* std_dev_p: " << std_dev_p << std::endl;
  std::cout << "* std_dev_a: " << std_dev_a << std::endl;
  std::cout << "* std_dev_p_rel: " << std_dev_a << std::endl;
  std::cout << "* omega: " << std_dev_a << std::endl;


  typedef std::shared_ptr<SimInstanceDelay> ptr_Instance;
  std::unordered_map<int, ptr_Instance> dict_instance;

  for(int i=0; i < num_instances; i++) {
    size_t ID = i;
    dict_instance[i] = ptr_Instance(new SimInstanceDelay(ID, dt, D, omega, std_dev_p, std_dev_a, std_dev_p_rel, delay_private, delay_joint, ptr_Handler));
    ptr_Handler->add(dict_instance[i]->ptr_IKF);

    if (first_private_only && i > 0) {
      dict_instance[i]->perform_private = false;
    }
  }

  if(joint_updates && num_instances > 1) {
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

  // we need to compute the estimated trajectory based on the final HistBelief!
  for(int i=0; i < num_instances; i++) {
    dict_instance[i]->calc_est_traj();
  }

  if(list_beliefs) {
    for(int i=0; i < num_instances; i++) {
      std::cout << "MeasData of filter instance ID=" << dict_instance[i]->ptr_IKF->ID() << std::endl;
      dict_instance[i]->ptr_IKF->print_HistMeas(20);
    }

    for(int i=0; i < num_instances; i++) {
      std::cout << "Beliefs of filter instance ID=" << i << std::endl;
      dict_instance[i]->print_HistBelief(20);
    }
  }
  if (show_plots) {
    for(int i=0; i < num_instances; i++) {
      dict_instance[i]->traj.plot_trajectory(i, "D-True");
      dict_instance[i]->traj_est.plot_trajectory(i, "D-Est");
      dict_instance[i]->compute_error();
      dict_instance[i]->traj_err.plot_trajectory(i, "D-Err");
    }

    wait_for_key();
  }
  else {
    for(int i=0; i < num_instances; i++) {
      dict_instance[i]->compute_error();
    }
  }

  return 0;
}
