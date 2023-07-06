/******************************************************************************
* FILENAME:     UnivariateNormal.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     06.07.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_UNIVARIATENORMAL_HPP
#define IKF_UNIVARIATENORMAL_HPP
#include <ikf/ikf_api.h>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <ikf/utils/rand_op.hpp>

namespace ikf {


template <typename Scalar>
class IKF_API UnivariateNormal {
  double  m_mean = 0;
  double  m_std_dev = 1;

public:
  UnivariateNormal(double const mean = 0,
                   double const std_dev = 1)
      : m_mean(mean), m_std_dev(std_dev) {}

  void setSeed(const uint64_t seed) {
    ikf::GaussianNoiseGen& gen = ikf::GaussianNoiseGen::instance();
    gen.seed(seed);
  }
  void setMean(double const mean) { m_mean = mean; }
  void setCovar( double const std_dev) {  m_std_dev = std_dev; }

  /// Draw nn samples from the gaussian and return them
  /// as columns in a Dynamic by nn matrix
  Eigen::Array<Scalar, 1, Eigen::Dynamic> samples(int const nn) {
    ikf::GaussianNoiseGen& gen = ikf::GaussianNoiseGen::instance();
    return (m_std_dev * Eigen::Array<Scalar, 1, Eigen::Dynamic>::NullaryExpr(1, nn, rand_op<Scalar>(gen))) + m_mean;
  }
};  // end class UnivariateNormal


} // ns ikf

#endif // IKF_UNIVATIATENORMAL_HPP
