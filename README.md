# ikf_lib (Isolated Kalman Filtering C++ library) 


This repository contains an **Isolated Kalman Filtering** framework, implementing the isolated Kalman filter (IKF) paradigm with support for out-of-sequence measurements. The primary motivation of the IKF paradigm is to decouple correlated estimates of filter instances by employing approximations proposed by Luft et al. in [1]. For preliminaries, details on the naming convention, and an overall overview of related algorithms, please refer to [5,  6].
Out-of-sequence measurements require a reprocessing of later and already processed measurements. Reprocessed measurements are prioritized: proprioceptive measurements over private, and private over joint measurements, to achieve best performance, which might be still suboptimal due to the approximations made during isolated joint measurements.
Note that the order of concurent measurements (measurements at orignating at the same point in time) matters -- in contrast to the Kalman filter, where updates at a specific time can be processed in either order, as all information is available.

Properties of the paradigm can be summarized as follows:
1. Execution order of **isolated** updates due to concurrent measurements matters. Preferable: propagation > private > joint. Not only per instance, but globally/ among all instances. (To achieve that, information exchange is needed to detect a violation and to correct violated priorities). Further, if multiple joint observations are performed at a single time instance, an optimal order can be found (e.g. prioritize common roots).
1. Out-of-sequence / delayed measurememts, need to consider (1) the priority at the current timestamp among all instances and (2) all measurements from all instances need to be processed chronologically sorted and prioritized synchronously, e.g., triggered from a interim master or central entity (instance handler).
1. Handling delayed measurements only across participants is degrading the estimation performance and if a participant was participating in another joint observation after the delayed one, this observation would be ignored (as the other interim master will not be triggered to redo updates)! Meaning this would only work if joint measurements would have a similar/equal delay and are not concurrent. A naive solution is to redo all upates on all instances, or only among post-correlated instances (those who had joint updates after the delayed measurement happened, like an avalanche). The an optimal solution is currently implemented using the [ikf::IsolatedKalmanFilterHandler](./source/ikf/include/ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp), while a suboptimal solution for inter-agent observations is implemented using the [ikf::CollaborativeIKFHandler](./source/ikf/include/ikf/EstimatorHandler/CollaborativeIKFHandler.hpp). 
1. If cyclic joint measurements happen concurrently, e.g., H_{12}, H_{23}, H_{31} at t=1, then a tripple joint isolated observation is preferable over three isolated bi-joint observations, to reduce the effects of the approximations made in the individual joint observation.
1. The isolated Kalman filter, in case of delayed measurements, might loosing it's attribute of being decoupled, as persistent all-to-all communication and a centralized fusion logic might be needed. Additionally, it is preferalbe that private observations are less delayed than joint observations.


Find the libary impementation in `source/ikf` and the test directory in `source/tests/ikf-test`.
Find a commandline example tool for non-delayed measurements in `source/examples/ikf_simple_cmd` and the same for delayed measurements in `source/examples/ikf_delay_cmd`.

## Prerequisite

```
sudo apt update
sudo apt install build-essential cmake -y
sudo apt install libspdlog-dev -y
```

## Build project

```
git clone <url> ikf_lib             
cd ikf_lib && mkdir build && cd build
cmake ..  -DCMAKE_BUILD_TYPE=Release -DOPTION_BUILD_EXAMPLE=ON -DOPTION_BUILD_TESTS=ON -DOPTION_BUILD_EXAMPLES=ON -DBUILD_SHARED_LIBS=ON
make all test -j 1
```

(if the build failes, please try to re-run `make`; source code is downloaded automatically in the background) 

View the documentation in your browser: `build/doc/index.html`.


Testet under `Ubunut 20.04` with `gcc`  version `(Ubuntu 9.4.0-1ubuntu1~20.04.1) 9.4.0`. 
Note `C++17` is required in examples, due to [matplotplusplus](https://github.com/alandefreitas/matplotplusplus) for visualization. Without examples, the library works perfectly well with `C++11` standard. 
Change the option in [CompileOptions.cmake](./cmake/CompileOptions.cmake), if you want/need, but then build the library without examples.

## Run examples

Example is based on constant accelation moving body model in 1D (see https://www.kalmanfilter.net/modeling.html). The body is moving in a harmonic way and the filter obtains as control input  noisy acceleration measurements for the state propagation. The filter is corrected via noisy position updates and relative position updates between bodies.   

Please note that `ikf_simple_cmd` and `ikf_delay_cmd` should perform, up to the last measurement minus delay, equally (last delayed measurement are not processed in `ikf_delay_cmd`, thus depends on the set delay). See the provided beliefs in the beginning of the terminal output (`--list_beliefs`). In the simulation, each filter instance obtain measurements concurrently: ID0: propagation, private, and joint, the others: progation and joint observations. Globally seen, all propgations are provided first, then the updates. Meaning the simulation defines the order of updates, which in case of an Kalman filter would be irrelvant, while in the IKF, the order matters. 

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
Updates can also be processed centralized-equvivalent (` --centalized_equvivalent`).

```
build$ ./ikf_dealy_cmd -h
ikf_delay_cmd
Usage: /home/jungr/workspace/CNS/build-ikf_lib-Desktop-Debug/ikf_delay_cmdd [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
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
  --centalized_equvivalent BOOLEAN=0
                              Choose a IKFHandler (false) or the DecoupledPropagationHandler (true), which performs centalized-equvivalent update steps, thus exact.
```

## How to "use" the Isolated Kalman Filtering framework

The `Isolated Kalman Filter` can be used in estimation problems, where individual Kalman Filters have decoupled inputs (are able to perform state propagation independently), but their outputs are sporadically coupled (observations relate to two or more Kalman filters' states). A domain would be e.g. multi-agent/robot state estimation [2] or modular multi-sensor fusion [3,4]. As the [ikf::IIsolatedKalmanFilter](./source//ikf/include/ikf/Estimator/IIsolatedKalmanFilter.hpp) instances are currently maintained **centralized** in the [ikf::IIsolatedKalmanFilterHandler](./source//ikf/include/ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp), the framework is well suited for modular multi-sensor fusion. For multi-agent state estimation, one might aim to run filter instances in individual process to exchange information via (wireless) networks, meaning that a specialization of the [ikf::CollaborativeIKFHandler](./source//ikf/include/ikf/EstimatorHandler/CollaborativeIKFHandler.hpp) and communication middlelay realizing the interface of [ikf::IMultiAgentHandler](./source//ikf/include/ikf/EstimatorHandler/IMultiAgentHandler.hpp) using, e.g., the Robot Operation System (ROS), is required.  

A simple, linar example is implemented in [LinearIKF_1D_const_acc](./source/examples/ikf_delay_cmd/include/LinearIKF_1D_const_acc.hpp), where the virtual methods of [ikf::IIsolatedKalmanFilter](./source//ikf/include/ikf/Estimator/IIsolatedKalmanFilter.hpp) (`progapation_measurement`, `local_private_measurement`, `local_joint_measurement`)  are defined. As measurement/input we use a generic structure [MeasData](./source//ikf/include/ikf/Measurement/MeasData.hpp), providing meta information and is inevitable for (re)processing delayed measurements. In [SimInstanceDelay](./source/examples/ikf_delay_cmd/include/SimInstanceDelay.hpp) one can see how this structure is filled and provided to the `process_measurement` method of the filter instance. Please note, that the filter needs inputs to perform a state prediction in order to advance the state in time. If you do not have control inputs, provide simply a "empty"  measurement with the observation type `ikf::eObservationType::PRIVATE_OBSERVATION` and the timestamp `t_m` to which the state should be predicted, before a update measurement is issued/provided to the filter. 


Please note: redoing updates exactly requires a central measurement handling in the correct order (first propagations then updates across all filter instances!). 
If reprocessing would be triggered by an IKF instance,  that again sequentially triggers other instances is sub-optimal. One could chronologically sort **all** measurements from **all** instances after the delayed measurement and process them sequentially, but measurements need to be prioritized by their type. In the current realization one can choose if the measurememts are maintained and handled either in the [ikf::IIsolatedKalmanFilter](./source//ikf/include/ikf/Estimator/IIsolatedKalmanFilter.hpp) instances or in the [ikf::IIsolatedKalmanFilterHandler](./source//ikf/include/ikf/EstimatorHandler/IsolatedKalmanFilterHandler.hpp) by setting the flag `--meas_centralized`. Still measurements are computed isolated among participants and not **exactly** as a centralized-equvivalent estimator. 

Please note, that a **centralized-equvivalent fusion** across local IKF instances can be achieved using the [ikf::DecoupledPropagationHandler](./source//ikf/include/ikf/EstimatorHandler/DecoupledPropagationHandler.hpp), allowing, e.g, to realize a modular fusion approach that exploits the sparsity in the input coupling of the instances. 

Due to isolated filtering, the estimation results depends on the order measurements are processed at a certain timestamp, e.g., at t=5 we have three concurrent measurements: propagation, private, and a joint measurement. If the private is performed after the joint measurement, the estimation result is worse as compared to centralized estimator, as the correction obtained by the private measurement is not influencing the other participating filter instance. Delayed measurement handling cannot compensate the compulsury prioritization, if private measurements are more delayed than joint observations, by just redoing measurements after that event. Therefore, all measurements including those from that event need to be reprocessed. 


## Third party content:

* [spdlog](https://github.com/gabime/spdlog)
* [eigenmvn](https://github.com/beniz/eigenmvn)
* [MaRS](https://github.com/aau-cns/mars_lib)
* [matplotplusplus](https://github.com/alandefreitas/matplotplusplus)
* [CLI11](https://github.com/CLIUtils/CLI11)


## TODOs:

* EKF example using wheel robots in 2D.

# REFERENCES

*  [1] Luft, Lukas and Schubert, Tobias and Roumeliotis, Stergios I. and Burgard, Wolfram, "Recursive decentralized localization for multi-robot systems with asynchronous pairwise communication", 2018, IJRR
*  [2] Jung, Roland and Weiss, Stephan, "Scalable Recursive Distributed Collaborative State Estimation for Aided Inertial Navigation", Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), IEEE, Xi’an, DOI: 10.1109/ICRA48506.2021.9561203, 2021.
*  [3] Jung, Roland and Weiss, Stephan, "Modular Multi-Sensor Fusion: A Collaborative State Estimation Perspective", IEEE Robotics and Automation Letters, DOI: 10.1109/LRA.2021.3096165, 2021.
*  [4] Brommer, Christian and Jung, Roland and Steinbrener, Jan and Weiss, Stephan, “MaRS: A modular and robust sensor-fusion framework,” IEEE Robotics and Automation Letters, DOI: 10.1109/LRA.2020.3043195, 2020.
*  [5] Jung, Roland and Luft, Lukas and Weiss, Stephan, "The Isolated Kalman Filtering Paradigm", 2023
*  [6] Jung, Roland, “Recursive distributed collaborative aided inertial navigation,” Ph.D. dissertation, Faculty of Technical Sciences with the Control of Networked Systems group, University of Klagenfurt, Aug. 2023. 
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
@inproceedings{jung_isolated_2023,
   author   = {Jung, Roland and Luft, Lukas and Weiss, Stephan},
   journal  = {tbd},
   doi      = {tbd},
   title    = {The Isolated Kalman Filtering Paradigm},
   year     = {2023},
}
```
