# ikf_lib (Isolated Kalman Filtering C++ library) 


This repository contains an **Isolated Kalman Filtering** framework, implementing the isolated Kalman filter (IKF) paradigm with support for out-of-sequence measurements. The primary motivation of the IKF paradigm is to decouple correlated estimates of filter instances by employing approximations proposed by Luft et al. in [1]. For further details on the naming convention and algorithm, please refer to [3-4].
Out-of-sequence measurements require a reprocessing of later and already processed measurements. Reprocessed measurements are prioritized: proprioceptive measurements over private, and private over joint measurements, to achieve best performance, which is still suboptimal due to the approximations made in the paradigm.
Harsh conditions for the paradigm are circular joint measurements without private measurements, and concurrent measurements, as the order of measurements at a specific point in time matters -- in contrast to the Kalman filter, where updates at a specific time can be processed in either order, as all information is available.

Properties of the paradigm can be summarized as follows:
1. Execution order of **isolated** updates due to concurrent measurements matters. Preferable: propagation > private > joint. Not only per instance, but globally/ among all instances. (To achieve that, information exchange is needed to detect a violation and to correct violated priorities). Further, if multiple joint observations are performed at a single time instance, an optimal order can be found (e.g. prioritize common roots).
1. Out-of-sequence / delayed measurememts, need to consider (1) the priority at the current timestamp among all instances and (2) all measurements from all instances need to be processed chronologically sorted and prioritized synchronously, e.g., triggered from a interim master or central entity (instance handler).
1. Handling delayed measurements only across participants is degrading the estimation performance and if a participant was participating in another joint observation after the delayed one, this observation would be ignored (as the other interim master will not be triggered to redo updates)! Meaning this would only work if joint measurements would have a similar/equal delay and are not concurrent. A naive solution is to redo all upates on all instances, or only among post-correlated instances (those who had joint updates after the delayed measurement happened, like an avalanche). The naive solution is currently implemented using the [ikf::IsolatedKalmanFilterHandler](./source/ikf/include/ikf/Estimator/IsolatedKalmanFilterHandler.hpp). 
1. If cyclic joint measurements happen concurrently, e.g., H_{12}, H_{23}, H_{31} at t=1, then a tripple joint isolated observation is preferable over three isolated bi-joint observations, to reduce the effects of the approximations made in the individual joint observation.
1. The isolated Kalman filter, in case of delayed measurements, is loosing it's attribute of being decoupled, as persistent all-to-all communication and a centralized fusion logic is needed! Therefore, one might want to make **further assumptions**: Joint updates are more delayed than private updates. To ignore the concurrent order of private and joint updates and assume that updates reoccure frequently, such that the effect of priortizing concurrent measurements vanishes. Additionally, one might not trigger redo updates synchronously and among all correlated instances. Starting from participants, one might trigger all other later (post-correlated) participants of joint updates (in the limit all instances). 


Find the libary impementation in `source/ikf` and the test directory in `source/tests/ikf-test`.
Find a commandline example tool for non-delayed measurements in `source/examples/ikf_simple_cmd` and the same for delayed measurements in `source/examples/ikf_delay_cmd`.

## Prerequisite

```
sudo apt install libspdlog-dev
```

## Build project

```
mkdir build && cd build
cmake .. -DOPTION_BUILD_EXAMPLE=ON -DOPTION_BUILD_TESTS=ON -DOPTION_BUILD_EXAMPLES=ON -DBUILD_SHARED_LIBS=ON
make all test -j 1
```

(if the build failes, please try to re-run `make`; source code is downloaded automatically in the background) 

View the documentation in your browser: `build/doc/index.html`.


Testet under `Ubunut 20.04` with `gcc`  version `(Ubuntu 9.4.0-1ubuntu1~20.04.1) 9.4.0`. 
Note `C++17` is required in examples, due to [matplotplusplus](https://github.com/alandefreitas/matplotplusplus) for visualization. Without examples, the library works perfectly well with `C++11` standard. 
Change the option in [CompileOptions.cmake](./cmake/CompileOptions.cmake), if you want/need, but then build the library without examples.

## Run examples

Example is based on constant accelation moving body model in 1D (see https://www.kalmanfilter.net/modeling.html). The body is moving in a harmonic way and the filter obtains as control input  noisy acceleration measurements for the state propagation. The filter is corrected via noisy position updates and relative position updates between bodies.   

Please note that `ikf_simple_cmd` and `ikf_delay_cmd` should perform, up to the last measurement minus delay, equally (last delayed measurement are not processed in `ikf_delay_cmd`, thus depends on the set delay). See the provided Beliefs in the beginning in the terminal output (`--list_beliefs`). In the simulation, each filter instance obtain measurements concurrently: ID0: propagation, private, and joint, the others: progation and joint observations. Globally seen, all propgations are provided first, then the updates. Meaning the simulation defines the order of updates, which in case of an Kalman filter would be irrelvant, while in the IKF, the order matters. 

### ikf_simple_cmd

Naive implementation of the IKF, without supporting delayed measurements (no buffering). 

```
build$ ./ikf_delay_cmd --help
ikf_simple_cmd
Usage: ./ikf_simple_cmdd [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --num_instances INT=4       number of filter instances
  --duration INT=5            Duration of the trajectory [sec]
  --first_private_only BOOLEAN=1
                              specifies if the first instance is obtaining private observations only
  --joint_updates BOOLEAN=1   specifies if cyclic joint observations are performed (ID_J = (ID_I + 1) % num_instances)
  --list_beliefs BOOLEAN=0    show a list of bliefs
  --show_plots BOOLEAN=1      show plots of the estimated trajectories
  --seed INT=123123           seed of random number generator
  --frequency INT=100         frequency of propagations
  --omega FLOAT=1.5708        omega, angular frequency of harominc
  --std_dev_p FLOAT=0.05      position measurement noise
  --std_dev_a FLOAT=0.05      acceleration input noise
  --std_dev_p_rel FLOAT=0.05  relative position measurement noise
```

### ikf_delay_cmd

Implementation of the IKF supporting delayed measurements.
Measurements can be maintained either in the IKF instances or in the IKF-Handler (`--meas_centralized`). Either should perform equal to the `ikf_delay_cmd`. 

```
build$ ./ikf_dealy_cmd -h
ikf_delay_cmd
Usage: ./ikf_delay_cmd [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --meas_centralized BOOLEAN=0
                              specifies if measurements are maintained in handler or individual filter instances
  --num_instances INT=4       number of filter instances
  --duration INT=5            Duration of the trajectory [sec]
  --first_private_only BOOLEAN=1
                              specifies if the first instance is obtaining private observations only
  --joint_updates BOOLEAN=1   specifies if cyclic joint observations are performed (ID_J = (ID_I + 1) % num_instances)
  --delay_private INT=1       number of steps private measurements are delayed 
  --delay_joint INT=1         number of steps joint measurements are delayed 
  --list_beliefs BOOLEAN=0    show a list of bliefs
  --show_plots BOOLEAN=1      show plots of the estimated trajectories
  --seed INT=123123           seed of random number generator
  --frequency INT=100         frequency of propagations
  --omega FLOAT=1.5708        omega, angular frequency of harominc
  --std_dev_p FLOAT=0.05      position measurement noise
  --std_dev_a FLOAT=0.05      acceleration input noise
  --std_dev_p_rel FLOAT=0.05  relative position measurement noise
```

## How to "use" the Isolated Kalman Filtering framework

The `Isolated Kalman Filter` can be used in estimation problems, where individual Kalman Filters have decoupled inputs (are able to perform state propagation independently), but their outputs are sporadically coupled (observations relate to two or more Kalman filters' states). A domain would be e.g. multi-agent/robot state estimation or modular multi-sensor fusion. As the [ikf::IIsolatedKalmanFilter](./source//ikf/include/ikf/Estimator/IIsolatedKalmanFilter.hpp) instances are currently maintained **centralized** in the [ikf::IIsolatedKalmanFilterHandler](./source//ikf/include/ikf/Estimator/IsolatedKalmanFilterHandler.hpp), the framework is well suited for modular multi-sensor fusion. For multi-agent state estimation, one might aim to run filter instances in individual process to exchange information via (wireless) networks, meaning that a specialization of the [ikf::IIsolatedKalmanFilterHandler](./source//ikf/include/ikf/Estimator/IsolatedKalmanFilterHandler.hpp) and communication middlelay, e.g., the Robot Operation System (ROS), is required.  

A simple, linar example is implemented in [LinearIKF_1D_const_acc](./source/examples/ikf_delay_cmd/include/LinearIKF_1D_const_acc.hpp), where the virtual methods of [ikf::IIsolatedKalmanFilter](./source//ikf/include/ikf/Estimator/IIsolatedKalmanFilter.hpp) (`progapation_measurement`, `local_private_measurement`, `local_joint_measurement`)  are defined. As measurement/input we use a generic structure [MeasData](./source//ikf/include/ikf/Measurement/MeasData.hpp), providing meta information and is inevitable for (re)processing delayed measurements. In [SimInstanceDelay](./source/examples/ikf_delay_cmd/include/SimInstanceDelay.hpp) one can see how this structure is filled and provided to the `process_measurement` method of the filter instance. Please note, that the filter needs inputs to perform a state prediction in order to advance the state in time. If you do not have control inputs, provide simply a "empty"  measurement with the observation type `ikf::eObservationType::PRIVATE_OBSERVATION` and the timestamp `t_m` to which the state should be predicted, before a update measurement is issued/provided to the filter. 


Please note: redoing updates exactly requires a central measurement handling in the correct order (first propagations then updates across all filter instances!). 
If reprocessing would be triggered by an IKF instance,  that again sequentially triggers other instances is sub-optimal. One could chronologically sort **all** measurements from **all** instances after the delayed measurement and process them sequentially, but propagation measurements need to be prioritized, which is not straightforward (multimap has no order for elements at a key, just fifo). In the current realization one can choose if the measurememts are maintained and handled either **distributed** in the [ikf::IIsolatedKalmanFilter](./source//ikf/include/ikf/Estimator/IIsolatedKalmanFilter.hpp) instances or **centralized** in the [ikf::IIsolatedKalmanFilterHandler](./source//ikf/include/ikf/Estimator/IsolatedKalmanFilterHandler.hpp) by setting the flag `--meas_centralized`. Still measurements are computed isolated among participants and not **exactly** as a centralized-equvivalent estimator. 

Due to isolated filtering, the estimation results depends on the order measurements are processed at a certain timestamp, e.g. at t=5 we have three concurrent measurements: propagation, private, and a joint measurement. If the private is performed after the joint measurement, the estimation result is worse as compared to centralized estimator, as the correction obtained by the private measurement is not influencing the other participating filter instance. Delayed measurement handling cannot compensate the compulsury prioritization, if private measurements are more delayed than joint observations, by just redoing measurements after that event. Therefore, all measurements including those from that event need to be reprocessed. 

## TODO:

* Support delayed updates between two prediction steps.
* Redo-update avalanche for participants and post-correlated instances after t0, by triggering all agents with interin joint measurements after t0 to redo their updates (currently all are redoing updates).

## Third party content:

* [spdlog](https://github.com/gabime/spdlog)
* [eigenmvn](https://github.com/beniz/eigenmvn)
* [matplotplusplus](https://github.com/alandefreitas/matplotplusplus)
* [CLI11](https://github.com/CLIUtils/CLI11)


# REFERENCES

*  [1] Luft, Lukas and Schubert, Tobias and Roumeliotis, Stergios I. and Burgard, Wolfram, "Recursive decentralized localization for multi-robot systems with asynchronous pairwise communication", 2018, IJRR
*  [2] Jung, Roland and Weiss, Stephan, "Scalable Recursive Distributed Collaborative State Estimation for Aided Inertial Navigation", Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), IEEE, Xi’an, DOI: 10.1109/ICRA48506.2021.9561203, 2021.
*  [3] Jung, Roland and Weiss, Stephan, "Modular Multi-Sensor Fusion: A Collaborative State Estimation Perspective", IEEE Robotics and Automation Letters, DOI: 10.1109/LRA.2021.3096165, 2021.
*  [4] Jung, Roland and Luft, Lukas and Weiss, Stephan, "The Isolated Kalman Filtering Paradigm", 2023

# CREDITS

Cmake project based on [cginternals/cmake-init](https://github.com/cginternals/cmake-init).


# License

This software is made available to the public to use (_source-available_), 
licensed under the terms of the BSD-2-Clause-License with no commercial use 
allowed, the full terms of which are made available in the `LICENSE` file. 
No license in patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@inproceedings{jung_isolated_23,
   author   = {Jung, Roland and Luft, Lukas and Weiss, Stephan},
   journal  = {To be defined},
   title    = {The Isolated Kalman Filtering Paradigm},
   year     = {2023},
}
```
