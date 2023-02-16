/******************************************************************************
* FILENAME:     eigen_utils.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
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
#ifndef EIGEN_UTILS_HPP
#define EIGEN_UTILS_HPP
#include <ikf/ikf_api.h>
#include <eigen3/Eigen/Eigen>

namespace ikf{
namespace utils {

template<typename scalar>
static std::vector<scalar> to_vector(Eigen::Array<scalar, Eigen::Dynamic, 1> const & vec) {
  std::vector<scalar> v(vec.rows());
  for(size_t i = 0; i < static_cast<size_t>(vec.rows()); i++) {
    v.at(i) = vec(i);
  }
  return v;
}

template<typename scalar>
static std::vector<scalar> to_vector(Eigen::Array<scalar, 1, Eigen::Dynamic> const & vec) {
  std::vector<scalar> v(vec.cols());
  for(size_t i = 0; i < static_cast<size_t>(vec.cols()); i++) {
    v.at(i) = vec(i);
  }
  return v;
}


bool IKF_API negative_diag(Eigen::MatrixXd const& A);

bool IKF_API is_symmetric(Eigen::MatrixXd const& A);

bool IKF_API is_positive_semidefinite(Eigen::MatrixXd const& A);

Eigen::MatrixXd IKF_API horcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ);

Eigen::MatrixXd IKF_API vertcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ);

Eigen::VectorXd IKF_API vertcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ);
///
/// \brief stabilize_covariance
/// \param Sigma
/// \param eps
/// \return
///
Eigen::MatrixXd IKF_API stabilize_covariance(Eigen::MatrixXd const& Sigma, double const eps=1e-12);

Eigen::MatrixXd IKF_API symmetrize_covariance(Eigen::MatrixXd const& Sigma);


template <typename Derived>
static std::string get_shape(const Eigen::EigenBase<Derived>& x)
{
  std::ostringstream oss;
  oss  << "[" << x.rows() << "x" << x.cols() << "]";
  return oss.str();
}

} // namespace utils
} // namespace ikf

#endif // EIGEN_UTILS_HPP
