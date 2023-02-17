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
build$ ./ikf_simple_cmd
```

### ikf_delay_cmd

Implementation of the IKF supporting delayed measurements.

```
build$ ./ikf_dealy_cmd
```

## How to "use" the Isolated Kalman Filtering framework

The `Isolated Kalman Filter` can be used in estimation problems, where individual Kalman Filters have decoupled inputs (are able to perform state propagation independently), but their outputs are sporadically coupled (observations relate to two or more Kalman filters' states). A domain would be e.g. multi-agent/robot state estimation or modular multi-sensor fusion.   

A simple, linar example is implemented in [LinearIKF_1D_const_acc](./source/examples/ikf_delay_cmd/include/LinearIKF_1D_const_acc.hpp), where the virtual methods of [ikf::IIsolatedKalmanFilter](./source//ikf/include/ikf/Estimator/IIsolatedKalmanFilter.hpp) (`progapation_measurement`, `local_private_measurement`, `local_joint_measurement`)  are defined. As measurement/input we use a generic structure [MeasData](./source//ikf/include/ikf/Measurement/MeasData.hpp), providing meta information and is inevitable for (re)processing delayed measurements. In [SimInstanceDelay](./source/examples/ikf_delay_cmd/include/SimInstanceDelay.hpp) one can see how this structure is filled and provided to the `process_measurement` method of the filter instance. Please note, that the filter needs inputs to perform a state prediction in order to advance the state in time. If you do not have control inputs, provide simply a "empty"  measurement with the observation type `ikf::eObservationType::PRIVATE_OBSERVATION` and the timestamp `t_m` to which the state should be predicted, before a update measurement is issued/provided to the filter. 

## Third party content:

* [eigenmvn](https://github.com/beniz/eigenmvn)
* [matplotplusplus](https://github.com/alandefreitas/matplotplusplus)


# REFERENCES

*  [1] Lukas Luft et al. "Recursive decentralized localization for multi-robot systems with asynchronous pairwise communication", 2018, IJRR
*  [2] Roland Jung and Stephan Weiss, "Scalable Recursive Distributed Collaborative State Estimation for Aided Inertial Navigation", Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), IEEE, Xi’an, DOI: 10.1109/ICRA48506.2021.9561203, 2021.
*  [3] Roland Jung and Stephan Weiss, "Modular Multi-Sensor Fusion: A Collaborative State Estimation Perspective", IEEE Robotics and Automation Letters, DOI: 10.1109/LRA.2021.3096165, 2021.
*  [4] Roland Jung,  Lukas Luft, and Stephan Weiss, "The Isolated Kalman Filtering Paradigm", 2023

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
   author   = {Roland Jung,  Lukas Luft, and Stephan Weiss},
   journal  = {},
   title    = {"The Isolated Kalman Filtering Paradigm},
   year     = {2023},
}
```
