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


## Third party content:

* [eigenmvn](https://github.com/beniz/eigenmvn)
* [matplotplusplus](https://github.com/alandefreitas/matplotplusplus)


# CREDITS

Cmake project based on [cginternals/cmake-init](https://github.com/cginternals/cmake-init).
