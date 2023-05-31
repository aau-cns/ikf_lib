/******************************************************************************
* FILENAME:     NormalizedInnovationSquared.hpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         <roland.jung@ieee.org>
* VERSION:      v0.0.0
* CREATION:     26.01.2023
*
* Copyright (C) 2023 Roland Jung, Control of Networked Systems, University of Klagenfurt, Austria.
*
* All rights reserved.
*
* This software is licensed under the terms of the BSD-2-Clause-License with
* no commercial use allowed, the full terms of which are made available
* in the LICENSE file. No license in patents is granted.
*
* You can contact the author at <roland.jung@aau.at>
******************************************************************************/
#ifndef NORMALIZEDINNOVATIONSQUARED_HPP
#define NORMALIZEDINNOVATIONSQUARED_HPP
#include <ikf/ikf_api.h>

#include <map>
#include <vector>
//#include <boost/math/distributions/chi_squared.hpp>
#include <Eigen/Dense>
#include <ikf/utils/eigen_utils.hpp>

#define SMALL_NIS_LOOKUP_TABLE 1

namespace ikf {

class IKF_API NormalizedInnovationSquared {
public:
  static const std::map<int, std::vector<double>> lookup;
  static bool check_dim(Eigen::VectorXd const& r, Eigen::MatrixXd const& Sigma);
  static double NIS(Eigen::MatrixXd const& Sigma, Eigen::VectorXd const& r);
  static bool check_NIS(Eigen::MatrixXd const& H, Eigen::MatrixXd const& R, Eigen::VectorXd const& r,
                        Eigen::MatrixXd const& Sigma, double confidence_interval=0.997);
  static bool check_NIS(Eigen::MatrixXd const& S, Eigen::VectorXd const& r, double confidence_interval=0.997);

  ///
  /// \brief chi2inv  returns the inverse cumulative distribution function (icdf) of the chi-square distribution with degrees of freedom nu, evaluated at the probability values in p.
  /// \param p  Probability values at which to evaluate icdf; a scalar value in [0,1]; optimized for [0.68, 0.95, 0.997];
  /// \param df  Degrees of freedom for the chi-square distribution, specified as a positive scalar value or an array of positive scalar values. optimized for [1:50];
  /// \return Finds the p percentile for the chi-square distribution with  degrees of freedom. If you generate random numbers from this chi-square distribution, you would observe numbers greater than the returned only (1-p)% of the time
  ///
  static double chi2inv(double const p, size_t const df);
};



} // namespace ikf
#endif // NORMALIZEDINNOVATIONSQUARED_HPP
