/******************************************************************************
* FILENAME:     NormalizedInnovationSquared.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include<ikf/Sensor/Estimator/NormalizedInnovationSquared.hpp>
#include <iostream>
namespace ikf {

bool NormalizedInnovationSquared::check_dim(const Eigen::VectorXd &r, const Eigen::MatrixXd &Sigma) {
  bool good = true;
  if ((r.cols() != 1) || (r.rows() != Sigma.rows())) {
    std::cout << "IKalmanFilter::check_dim(): residual must be a colum vector! r"<< utils::get_shape(r);
    std::cout << " and Sigma" << utils::get_shape(Sigma) << std::endl;
    good = false;
  }
  return good;
}

double NormalizedInnovationSquared::NIS(const Eigen::MatrixXd &Sigma, const Eigen::VectorXd &r) {
  if ( check_dim(r, Sigma) ) {
    Eigen::MatrixXd s =  r.transpose().eval() * Sigma.inverse() * r; // [1xN]*[NxN]*[Nx1] = [1x1]
    return s(0,0);
  } else {
    return 0.0;
  }
}

bool NormalizedInnovationSquared::check_NIS(const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Eigen::MatrixXd &Sigma, double confidence_interval) {
  Eigen::MatrixXd S = H*Sigma*H.transpose() + R;
  S = utils::symmetrize_covariance(S);
  double s = NIS(S, r);

  double s_threshold = NormalizedInnovationSquared::chi2inv(confidence_interval, r.rows());
  if (s > s_threshold) {
    return false;
  }
  return true;
}

bool NormalizedInnovationSquared::check_NIS(const Eigen::MatrixXd &S, const Eigen::VectorXd &r, double confidence_interval) {
  double s = NIS(S, r);
  double s_threshold = NormalizedInnovationSquared::chi2inv(confidence_interval, r.rows());
  if (s > s_threshold) {
    return false;
  }
  return true;
}

double NormalizedInnovationSquared::chi2inv(const double p, const size_t df) {
  // TODO: this method brings us the boost dependency: Technically we could use a table with thresholds.
  int p_ = round(p*1000);
  auto iter = lookup.find(p_);
  if (iter != lookup.end()) {
    if (iter->second.size() > df) {
      return iter->second.at(df);
    }
  }
  //return quantile(boost::math::chi_squared(df), p);
  return 0;
}

#if SMALL_NIS_LOOKUP_TABLE == 0
// MATLAB: T = generate_NIS_table([0.38, 0.5, 0.68, 0.95, 0.99, 0.997], [1:100]);
const std::map<int, std::vector<double>>  NormalizedInnovationSquared::lookup = {
    {380, {0.24587,0.95607,1.7768,2.6386,3.5224,4.4203,5.328,6.2433,7.1645,8.0905,9.0205,9.954,10.8904,11.8294,12.7707,13.7141,14.6594,15.6063,16.5548,17.5047,18.4559,19.4083,20.3618,21.3163,22.2719,23.2283,24.1856,25.1438,26.1026,27.0623,28.0226,28.9835,29.9451,30.9073,31.87,32.8333,33.7971,34.7614,35.7262,36.6914,37.6571,38.6232,39.5898,40.5567,41.524,42.4916,43.4597,44.428,45.3967,46.3658,47.3351,48.3048,49.2747,50.2449,51.2154,52.1862,53.1572,54.1285,55.1001,56.0719,57.0439,58.0162,58.9887,59.9614,60.9343,61.9074,62.8808,63.8543,64.8281,65.802,66.7761,67.7504,68.7249,69.6995,70.6744,71.6494,72.6245,73.5999,74.5754,75.551,76.5268,77.5027,78.4788,79.4551,80.4315,81.408,82.3846,83.3614,84.3384,85.3154,86.2926,87.2699,88.2474,89.2249,90.2026,91.1804,92.1583,93.1364,94.1145,95.0928}},
    {500, {0.45494,1.3863,2.366,3.3567,4.3515,5.3481,6.3458,7.3441,8.3428,9.3418,10.341,11.3403,12.3398,13.3393,14.3389,15.3385,16.3382,17.3379,18.3377,19.3374,20.3372,21.337,22.3369,23.3367,24.3366,25.3365,26.3363,27.3362,28.3361,29.336,30.3359,31.3359,32.3358,33.3357,34.3356,35.3356,36.3355,37.3355,38.3354,39.3353,40.3353,41.3352,42.3352,43.3352,44.3351,45.3351,46.335,47.335,48.335,49.3349,50.3349,51.3349,52.3348,53.3348,54.3348,55.3348,56.3347,57.3347,58.3347,59.3347,60.3346,61.3346,62.3346,63.3346,64.3346,65.3345,66.3345,67.3345,68.3345,69.3345,70.3345,71.3344,72.3344,73.3344,74.3344,75.3344,76.3344,77.3344,78.3343,79.3343,80.3343,81.3343,82.3343,83.3343,84.3343,85.3343,86.3342,87.3342,88.3342,89.3342,90.3342,91.3342,92.3342,93.3342,94.3342,95.3342,96.3342,97.3341,98.3341,99.3341}},
    {680, {0.98895,2.2789,3.5059,4.6954,5.8608,7.0092,8.1448,9.2704,10.388,11.4988,12.6039,13.7041,14.8001,15.8922,16.981,18.0668,19.1498,20.2304,21.3086,22.3848,23.459,24.5315,25.6022,26.6714,27.7392,28.8055,29.8706,30.9344,31.9971,33.0587,34.1193,35.1788,36.2374,37.2952,38.352,39.4081,40.4633,41.5178,42.5716,43.6247,44.6771,45.7289,46.78,47.8305,48.8805,49.9299,50.9788,52.0271,53.0749,54.1223,55.1691,56.2155,57.2615,58.307,59.3521,60.3967,61.441,62.4849,63.5284,64.5715,65.6143,66.6567,67.6988,68.7405,69.7819,70.823,71.8637,72.9042,73.9444,74.9842,76.0238,77.0631,78.1022,79.1409,80.1794,81.2177,82.2556,83.2934,84.3309,85.3681,86.4052,87.4419,88.4785,89.5149,90.551,91.5869,92.6226,93.6581,94.6934,95.7285,96.7634,97.7981,98.8326,99.867,100.9011,101.9351,102.9689,104.0025,105.0359,106.0692}},
    {950, {3.8415,5.9915,7.8147,9.4877,11.0705,12.5916,14.0671,15.5073,16.919,18.307,19.6751,21.0261,22.362,23.6848,24.9958,26.2962,27.5871,28.8693,30.1435,31.4104,32.6706,33.9244,35.1725,36.415,37.6525,38.8851,40.1133,41.3371,42.557,43.773,44.9853,46.1943,47.3999,48.6024,49.8018,50.9985,52.1923,53.3835,54.5722,55.7585,56.9424,58.124,59.3035,60.4809,61.6562,62.8296,64.0011,65.1708,66.3386,67.5048,68.6693,69.8322,70.9935,72.1532,73.3115,74.4683,75.6237,76.7778,77.9305,79.0819,80.2321,81.381,82.5287,83.6753,84.8206,85.9649,87.1081,88.2502,89.3912,90.5312,91.6702,92.8083,93.9453,95.0815,96.2167,97.351,98.4844,99.6169,100.7486,101.8795,103.0095,104.1387,105.2672,106.3948,107.5217,108.6479,109.7733,110.898,112.022,113.1453,114.2679,115.3898,116.511,117.6317,118.7516,119.8709,120.9896,122.1077,123.2252,124.3421}},
    {990, {6.6349,9.2103,11.3449,13.2767,15.0863,16.8119,18.4753,20.0902,21.666,23.2093,24.725,26.217,27.6882,29.1412,30.5779,31.9999,33.4087,34.8053,36.1909,37.5662,38.9322,40.2894,41.6384,42.9798,44.3141,45.6417,46.9629,48.2782,49.5879,50.8922,52.1914,53.4858,54.7755,56.0609,57.3421,58.6192,59.8925,61.1621,62.4281,63.6907,64.9501,66.2062,67.4593,68.7095,69.9568,71.2014,72.4433,73.6826,74.9195,76.1539,77.386,78.6158,79.8433,81.0688,82.2921,83.5134,84.7328,85.9502,87.1657,88.3794,89.5913,90.8015,92.01,93.2169,94.4221,95.6257,96.8278,98.0284,99.2275,100.4252,101.6214,102.8163,104.0098,105.202,106.3929,107.5825,108.7709,109.9581,111.144,112.3288,113.5124,114.6949,115.8763,117.0565,118.2357,119.4139,120.591,121.7671,122.9422,124.1163,125.2895,126.4617,127.6329,128.8032,129.9727,131.1412,132.3089,133.4757,134.6416,135.8067}},
    {997, {8.8075,11.6183,13.9314,16.0143,17.9576,19.8047,21.5801,23.2997,24.9741,26.6108,28.2156,29.7929,31.3461,32.878,34.3909,35.8868,37.3672,38.8335,40.2869,41.7283,43.1588,44.579,45.9897,47.3915,48.7849,50.1705,51.5487,52.9199,54.2845,55.6429,56.9953,58.342,59.6833,61.0194,62.3507,63.6772,64.9992,66.3168,67.6303,68.9397,70.2454,71.5472,72.8456,74.1404,75.4319,76.7202,78.0053,79.2875,80.5667,81.843,83.1166,84.3875,85.6559,86.9217,88.185,89.4459,90.7046,91.9609,93.2151,94.4671,95.717,96.9649,98.2108,99.4547,100.6967,101.9369,103.1753,104.4119,105.6467,106.8799,108.1113,109.3412,110.5695,111.7962,113.0214,114.245,115.4672,116.688,117.9074,119.1254,120.342,121.5572,122.7712,123.9839,125.1953,126.4055,127.6145,128.8222,130.0288,131.2342,132.4385,133.6417,134.8437,136.0447,137.2446,138.4435,139.6413,140.8381,142.0339,143.2287}}
};
#else
// MATLAB: T = generate_NIS_table([0.68, 0.95, 0.997], [1:50]);
const std::map<int, std::vector<double>>  NormalizedInnovationSquared::lookup = {
    {680, {0.98895,2.2789,3.5059,4.6954,5.8608,7.0092,8.1448,9.2704,10.388,11.4988,12.6039,13.7041,14.8001,15.8922,16.981,18.0668,19.1498,20.2304,21.3086,22.3848,23.459,24.5315,25.6022,26.6714,27.7392,28.8055,29.8706,30.9344,31.9971,33.0587,34.1193,35.1788,36.2374,37.2952,38.352,39.4081,40.4633,41.5178,42.5716,43.6247,44.6771,45.7289,46.78,47.8305,48.8805,49.9299,50.9788,52.0271,53.0749,54.1223}},
    {950, {3.8415,5.9915,7.8147,9.4877,11.0705,12.5916,14.0671,15.5073,16.919,18.307,19.6751,21.0261,22.362,23.6848,24.9958,26.2962,27.5871,28.8693,30.1435,31.4104,32.6706,33.9244,35.1725,36.415,37.6525,38.8851,40.1133,41.3371,42.557,43.773,44.9853,46.1943,47.3999,48.6024,49.8018,50.9985,52.1923,53.3835,54.5722,55.7585,56.9424,58.124,59.3035,60.4809,61.6562,62.8296,64.0011,65.1708,66.3386,67.5048}},
    {997, {8.8075,11.6183,13.9314,16.0143,17.9576,19.8047,21.5801,23.2997,24.9741,26.6108,28.2156,29.7929,31.3461,32.878,34.3909,35.8868,37.3672,38.8335,40.2869,41.7283,43.1588,44.579,45.9897,47.3915,48.7849,50.1705,51.5487,52.9199,54.2845,55.6429,56.9953,58.342,59.6833,61.0194,62.3507,63.6772,64.9992,66.3168,67.6303,68.9397,70.2454,71.5472,72.8456,74.1404,75.4319,76.7202,78.0053,79.2875,80.5667,81.843}}
};
#endif
}
