/******************************************************************************
* FILENAME:     eigen_utils.hpp
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
#ifndef EIGEN_UTILS_HPP
#define EIGEN_UTILS_HPP
#include <ikf/ikf_api.h>
#include <Eigen/Dense>
#include <iostream>

namespace ikf{
namespace utils {

template <typename scalar>
static std::vector<scalar> to_vector(Eigen::Array<double, Eigen::Dynamic, 1> const& vec) {
  // allocate and deep copy
  std::vector<scalar> v(vec.rows());
  for (size_t i = 0; i < static_cast<size_t>(vec.rows()); i++) {
    v.at(i) = (double)vec(i);
  }
  return v;
}

template<typename scalar>
static std::vector<scalar> to_vector(Eigen::Array<scalar, Eigen::Dynamic, 1> const & vec) {
  // allocate and deep copy
  std::vector<scalar> v(vec.rows());
  for(size_t i = 0; i < static_cast<size_t>(vec.rows()); i++) {
    v.at(i) = vec(i);
  }
  return v;
}

template <typename scalar>
static std::vector<scalar> to_vector(Eigen::Array<double, 1, Eigen::Dynamic> const& vec) {
  // allocate and deep copy
  std::vector<scalar> v(vec.cols());
  for (size_t i = 0; i < static_cast<size_t>(vec.cols()); i++) {
    v.at(i) = (scalar)vec(i);
  }
  return v;
}

template<typename scalar>
static std::vector<scalar> to_vector(Eigen::Array<scalar, 1, Eigen::Dynamic> const & vec) {
  // allocate and deep copy
  std::vector<scalar> v(vec.cols());
  for(size_t i = 0; i < static_cast<size_t>(vec.cols()); i++) {
    v.at(i) = vec(i);
  }
  return v;
}

template<typename scalar>
Eigen::Array<scalar, Eigen::Dynamic, 1> from_vector(std::vector<scalar> const & vec)
{
  // allocate and deep copy
  Eigen::VectorXd v(vec.size());
  for(size_t i = 0; i < vec.size(); i++) {
    v(i) = vec.at(i);
  }
  return v;
}

template<typename scalar>
Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic> toEigenMatrix(std::vector<std::vector<scalar>> const & mat)
{
  // allocate and deep copy
  size_t rows = mat.size();
  size_t cols = 1;
  if(rows) {
    cols = mat[0].size();
  }

  Eigen::MatrixXd m(rows, cols);
  for(size_t r = 0; r < rows;  r++) {
    for(size_t c = 0; c < cols;  c++) {
      m(r, c) = mat.at(r).at(c);
    }
  }
  return m;
}

template<typename scalar>
Eigen::Quaternion<scalar> toEigenQuat(Eigen::MatrixX<scalar> const& m) {
  Eigen::Quaternion<scalar> q;
  if(m.rows() == 4 && m.cols() == 1)
  {
    q.w() = m(0);
    q.x() = m(1);
    q.y() = m(2);
    q.z() = m(3);
  } else {
    std::cout << "eigen_utils::toEigenQuat(): conversion of " << m << " failed!" << std::endl;
  }
  return q;
}

template<typename scalar>
Eigen::Quaternion<scalar> toEigenQuat(Eigen::VectorX<scalar> const& m) {
  Eigen::Quaternion<scalar> q;
  if(m.rows() == 4 && m.cols() == 1)
  {
    q.w() = m(0);
    q.x() = m(1);
    q.y() = m(2);
    q.z() = m(3);
  } else {
    std::cout << "eigen_utils::toEigenQuat(): conversion of " << m << " failed!" << std::endl;
  }
  return q;
}

template<typename scalar>
Eigen::Quaternion<scalar> toEigenQuat(Eigen::Vector4<scalar> const& vec) {
  Eigen::Quaternion<scalar> q;
  q.w() = vec(0);
  q.x() = vec(1);
  q.y() = vec(2);
  q.z() = vec(3);
  return q;
}

template<typename scalar>
Eigen::Vector4<scalar> fromEigenQuat(Eigen::Quaternion<scalar> const& q) {
  Eigen::Vector4<scalar> vec;
  vec(0) = q.w();
  vec(1) = q.x();
  vec(2) = q.y();
  vec(3) = q.z();
  return vec;
}


template<typename scalar>
std::vector<std::vector<scalar>> fromEigenMatrix(Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic> const & M)
{
  std::vector<std::vector<scalar>> m;
  m.resize(M.rows(), std::vector<scalar>(M.cols(), 0));
  for(size_t i = 0; i < m.size(); i++)
    for(size_t j = 0; j < m.front().size(); j++)
      m[i][j] = M(i,j);
  return m;
}


bool IKF_API negative_diag(Eigen::MatrixXd const& A);

bool IKF_API is_symmetric(Eigen::MatrixXd const& A);

bool IKF_API is_positive_semidefinite(Eigen::MatrixXd const& A);

Eigen::MatrixXd IKF_API horcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ);
Eigen::MatrixXd IKF_API horcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &H_KK);
Eigen::MatrixXd IKF_API horcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &H_KK, const Eigen::MatrixXd &H_LL);

Eigen::MatrixXd IKF_API vertcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ);
Eigen::MatrixXd IKF_API vertcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &H_KK);
Eigen::MatrixXd IKF_API vertcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &H_KK, const Eigen::MatrixXd &H_LL);



Eigen::VectorXd IKF_API vertcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ);
Eigen::VectorXd IKF_API vertcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ,
                                    const Eigen::VectorXd &v_KK);

Eigen::VectorXd IKF_API horcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ);
Eigen::VectorXd IKF_API horcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ,
                                   const Eigen::VectorXd &v_KK);

Eigen::MatrixXd IKF_API stack_Sigma(const Eigen::MatrixXd& Sigma_II, const Eigen::MatrixXd& Sigma_JJ,
                                    const Eigen::MatrixXd& Sigma_IJ);

void IKF_API split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J,
                         Eigen::MatrixXd& Sigma_II, Eigen::MatrixXd& Sigma_JJ, Eigen::MatrixXd& Sigma_IJ);

void IKF_API split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J, size_t const dim_K,
                         Eigen::MatrixXd& Sigma_II, Eigen::MatrixXd& Sigma_JJ, Eigen::MatrixXd& Sigma_KK,
                         Eigen::MatrixXd& Sigma_IJ, Eigen::MatrixXd& Sigma_IK, Eigen::MatrixXd& Sigma_JK);

void IKF_API split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J, size_t const dim_K,
                         size_t const dim_L, Eigen::MatrixXd& Sigma_II, Eigen::MatrixXd& Sigma_JJ,
                         Eigen::MatrixXd& Sigma_KK, Eigen::MatrixXd& Sigma_LL, Eigen::MatrixXd& Sigma_IJ,
                         Eigen::MatrixXd& Sigma_IK, Eigen::MatrixXd& Sigma_JK, Eigen::MatrixXd& Sigma_IL,
                         Eigen::MatrixXd& Sigma_JL, Eigen::MatrixXd& Sigma_KL);
///
/// \brief stabilize_covariance
/// \param Sigma
/// \param eps
/// \return
///
Eigen::MatrixXd IKF_API stabilize_covariance(Eigen::MatrixXd const& Sigma, double const eps=1e-12);

Eigen::MatrixXd IKF_API symmetrize_covariance(Eigen::MatrixXd const& Sigma);

//

///
/// \brief Applies a eps on negative eigenvalues. Adapted from Christian Brommer:
/// https://github.com/aau-cns/mars_lib/blob/main/source/mars/source/nearest_cov.cpp
/// \param Sigma
/// \param eps
/// \return PSD Sigma
///
Eigen::MatrixXd IKF_API nearest_covariance(Eigen::MatrixXd const& Sigma, double const eps = 0.001);

template <typename Derived>
static std::string get_shape(const Eigen::EigenBase<Derived>& x)
{
  std::ostringstream oss;
  oss  << "[" << x.rows() << "x" << x.cols() << "]";
  return oss.str();
}

template <typename Derived=double>
static inline std::string print(Eigen::Quaternion<Derived> const& q)
{
  std::stringstream str;
  str << " (w,x,y,z) " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
  return str.str();
}

template <typename Derived=double>
static inline std::string print(Eigen::Matrix<Derived, 3, 1> const& p)
{
  std::stringstream str;
  str << " (x,y,z) " << p.x() << " " << p.y() << " " << p.z();
  return str.str();
}

} // namespace utils
} // namespace ikf

#endif // EIGEN_UTILS_HPP
