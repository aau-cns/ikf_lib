/******************************************************************************
* FILENAME:     MultivariateNormal.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     06.07.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
/**
 * Multivariate Normal distribution sampling using C++11 and Eigen matrices.
 *
 * This is taken from
 * http://stackoverflow.com/questions/16361226/error-while-creating-object-from-templated-class
 * (also see
 * http://lost-found-wandering.blogspot.fr/2011/05/sampling-from-multivariate-normal-in-c.html)
 *
 * I have been unable to contact the original author, and I've performed
 * the following modifications to the original code:
 * - removal of the dependency to Boost, in favor of straight C++11;
 * - ability to choose from Solver or Cholesky decomposition (supposedly
 * faster);
 * - fixed Cholesky by using LLT decomposition instead of LDLT that was not
 * yielding a correctly rotated variance (see this
 * http://stats.stackexchange.com/questions/48749/how-to-sample-from-a-multivariate-normal-given-the-pt-ldlt-p-decomposition-o
 * )
 */

/**
 * Copyright (c) 2014 by Emmanuel Benazera beniz@droidnik.fr, All rights reserved.
 * Copyright (c) 2023 by Roland Jung, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * source code base: https://github.com/beniz/eigenmvn
 */
#ifndef IKF_MULTIVARIATENORMAL_HPP
#define IKF_MULTIVARIATENORMAL_HPP
#include <ikf/ikf_api.h>
#include <ikf/utils/rand_op.hpp>

namespace ikf {

/**
  Find the eigen-decomposition of the covariance matrix
  and then store it for sampling from a multi-variate normal
*/
template <typename Scalar>
class IKF_API MultivariateNormal {
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> m_Sigma;
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> m_Tf;  // scales the random values according to the covariance m_Sigma
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> m_mean;

public:
  MultivariateNormal(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mean,
                     const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& covar,
                     const bool use_cholesky = false) {
    setMean(mean);
    setCovar(covar, use_cholesky);
  }
  virtual ~MultivariateNormal() = default;

  const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& getCovar() const { return m_Sigma; }
  void setSeed(const uint64_t seed) {
    ikf::GaussianNoiseGen& gen = ikf::GaussianNoiseGen::instance();
    gen.seed(seed);
  }
  void setMean(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& mean) { m_mean = mean; }

  void setCovar(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& covar, const bool use_cholesky = false) {
    //m_Sigma = 0.5 * (covar + covar.tr);
    m_Sigma = covar;
    // Assuming that we'll be using this repeatedly,
    // compute the transformation matrix that will
    // be applied to unit-variance independent normals
    if (use_cholesky) {
      Eigen::LLT<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> > solver(m_Sigma);
      // We can only use the cholesky decomposition if
      // the covariance matrix is symmetric, pos-definite.
      // But a covariance matrix might be pos-semi-definite.
      // In that case, we'll go to an EigenSolver
      if (solver.info() == Eigen::Success) {
        // Use cholesky solver
        m_Tf = solver.matrixL();
      } else {
        throw std::runtime_error("Failed computing the Cholesky decomposition. Use solver instead");
      }
    }
    else {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> > solver(m_Sigma);
      if (solver.info() == Eigen::Success) {
        m_Tf = solver.eigenvectors() * solver.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
      } else {
        throw std::runtime_error("Failed computing the SelfAdjointEigenSolver");
      }
    }
  }

  /// Draw nn samples from the gaussian and return them
  /// as columns in a Dynamic by nn matrix
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> samples(int const nn) {
    ikf::GaussianNoiseGen& gen = ikf::GaussianNoiseGen::instance();
    return (m_Tf *
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::NullaryExpr(m_Sigma.rows(), nn, rand_op<Scalar>(gen))).colwise() + m_mean;
  }
};  // end class MultivariateNormal

} // ns ikf


#endif // MULTIVARIATENORMAL_HPP
