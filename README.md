# Isolated Kalman Filtering framework 

[WIKI](https://gitlab.aau.at/aau-cns/aaucns_mmsf/wikis/home)

Find the lib impementation in `source/ikf` and the test directory in `source/tests/ikf-test`.
Find a commandline example tool in `source/examples/ikf_cmd`.


## build project

```
mkdir build && cd build
cmake .. -DOPTION_BUILD_EXAMPLE=ON -DOPTION_BUILD_TESTS=ON -DOPTION_BUILD_EXAMPLES=ON -DBUILD_SHARED_LIBS=ON
make all test
```

View the documentation in your browser: `build/doc/index.html`.

## run examples

Example is based on constant accelation moving body model in 1D (see https://www.kalmanfilter.net/modeling.html). The body is moving in a harmonic way and the filter obtains as control input  noisy acceleration measurements for the state propagation. The filter is corrected via noisy position updates and relative position updates between bodies.   

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
