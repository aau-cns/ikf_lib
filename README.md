# ikf_lib (Isolated Kalman Filtering C++ library) 

An **Isolated Kalman Filtering** framework. 


Find the lib impementation in `source/ikf` and the test directory in `source/tests/ikf-test`.
Find a commandline example tool in `source/examples/ikf_cmd`.


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

Please note that `ikf_simple_cmd` and `ikf_delay_cmd` should perform, up to the last measurement minus delay, equally (last delayed measurement are not processed in `ikf_delay_cmd`, thus depends on the set delay). See the provided Beliefs in the beginning in the terminal output.

### ikf_simple_cmd

Naive implementation of the IKF, without supporting delayed measurements (no buffering). 

```
build$ ./ikf_delay_cmd --help
ikf_simple_cmd
Usage: ./ikf_simple_cmdd [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --num_instances INT         number of filter instances
  --first_private_only BOOLEAN
                              specifies if the first instance is obtaining private observations only
  --joint_updates BOOLEAN     specifies if cyclic joint observations are performed (ID_J = (ID_I + 1) % num_instances)
  --list_beliefs BOOLEAN      show a list of bliefs
  --show_plots BOOLEAN        show plots of the estimated trajectories
```

### ikf_delay_cmd

Implementation of the IKF supporting delayed measurements.

```
build$ ./ikf_dealy_cmd -h
ikf_delay_cmd
Usage: ./ikf_delay_cmdd [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --meas_centralized BOOLEAN  specifies if measurements are maintained in handler or individual filter instances
  --num_instances INT         number of filter instances
  --first_private_only BOOLEAN
                              specifies if the first instance is obtaining private observations only
  --joint_updates BOOLEAN     specifies if cyclic joint observations are performed (ID_J = (ID_I + 1) % num_instances)
  --delay_private INT         number of steps private measurements are delayed 
  --delay_joint INT           number of steps joint measurements are delayed 
  --list_beliefs BOOLEAN      show a list of bliefs
  --show_plots BOOLEAN        show plots of the estimated trajectories
```

## How to "use" the Isolated Kalman Filtering framework

The `Isolated Kalman Filter` can be used in estimation problems, where individual Kalman Filters have decoupled inputs (are able to perform state propagation independently), but their outputs are sporadically coupled (observations relate to two or more Kalman filters' states). A domain would be e.g. multi-agent/robot state estimation or modular multi-sensor fusion.   

A simple, linar example is implemented in [LinearIKF_1D_const_acc](./source/examples/ikf_delay_cmd/include/LinearIKF_1D_const_acc.hpp), where the virtual methods of [ikf::IIsolatedKalmanFilter](./source//ikf/include/ikf/Estimator/IIsolatedKalmanFilter.hpp) (`progapation_measurement`, `local_private_measurement`, `local_joint_measurement`)  are defined. As measurement/input we use a generic structure [MeasData](./source//ikf/include/ikf/Measurement/MeasData.hpp), providing meta information and is inevitable for (re)processing delayed measurements. In [SimInstanceDelay](./source/examples/ikf_delay_cmd/include/SimInstanceDelay.hpp) one can see how this structure is filled and provided to the `process_measurement` method of the filter instance. Please note, that the filter needs inputs to perform a state prediction in order to advance the state in time. If you do not have control inputs, provide simply a "empty"  measurement with the observation type `ikf::eObservationType::PRIVATE_OBSERVATION` and the timestamp `t_m` to which the state should be predicted, before a update measurement is issued/provided to the filter. 


Please note: redoing updates exactly requires a central measurement handling in the correct order (first propagations then updates across all filter instances!). 
If reprocessing would be triggered by an IKF instance,  that again sequentially triggers other instances is sub-optimal. One could chronologically sort **all** measurements from **all** instances after the delayed measurement and process them sequentially, but propagation measurements need to be prioritized, which is not straightforward (multimap has no order for elements at a key, just fifo). Therefore, the current realization of the IKF with Handler and support of delayed measurements is not distributed, as it requires a central handler for
measurement distribution and re-computation (unfortunately). Still measurementscan be computed isolated among participants. 

## Third party content:

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
   journal  = {To be defined: IEEE Robotics and Automation Letters or ArXiv},
   title    = {The Isolated Kalman Filtering Paradigm},
   year     = {2023},
}
```
