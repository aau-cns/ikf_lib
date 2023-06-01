/******************************************************************************
* FILENAME:     eigen_utils.cpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     03.02.2023
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
#include <ikf/utils/eigen_utils.hpp>
#include <ikf/utils/RTVerification.hpp>

namespace ikf {

bool utils::negative_diag(const Eigen::MatrixXd &A) {
  Eigen::VectorXd diag = A.diagonal();
  return (diag.array() < 0.0).any();
}

bool utils::is_symmetric(const Eigen::MatrixXd &A) {
  return A.isApprox(A.transpose());
}

bool utils::is_positive_semidefinite(const Eigen::MatrixXd &A) {
  if (is_symmetric(A)) {
    Eigen::LDLT<Eigen::MatrixXd> ldltOfA(A);
    return (ldltOfA.info() != Eigen::NumericalIssue) && ldltOfA.isPositive();
  }
  return false;
}

Eigen::MatrixXd utils::horcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ) {
  RTV_EXPECT_TRUE_THROW(H_II.rows() == H_JJ.rows(), "dimension missmatch!");

  Eigen::MatrixXd C(H_II.rows(), H_II.cols()+H_JJ.cols());
  C.leftCols(H_II.cols()) = H_II;
  C.rightCols(H_JJ.cols()) = H_JJ;
  return C;
}

Eigen::MatrixXd utils::vertcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ) {
  RTV_EXPECT_TRUE_THROW(H_II.cols() == H_JJ.cols(), "dimension missmatch!");

  Eigen::MatrixXd C(H_II.rows()+H_JJ.rows(), H_JJ.cols());
  C << H_II,
      H_JJ;
  return C;
}

Eigen::VectorXd utils::vertcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ) {
  RTV_EXPECT_TRUE_THROW(v_II.cols() == v_JJ.cols(), "dimension missmatch!");

  Eigen::VectorXd C(v_II.rows()+v_JJ.rows(), v_JJ.cols());
  C << v_II,
      v_JJ;
  return C;
}

Eigen::VectorXd utils::horcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ) {
  RTV_EXPECT_TRUE_THROW(v_II.rows() == v_JJ.rows(), "dimension missmatch!");

  Eigen::VectorXd C(v_II.rows(), v_II.cols()+v_JJ.cols());
  C << v_II,
      v_JJ;
  return C;
}


Eigen::MatrixXd utils::stabilize_covariance(const Eigen::MatrixXd &Sigma, const double eps) {
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Sigma.rows(), Sigma.cols())*eps;
  return 0.5*(Sigma + Sigma.transpose()) + I;
}

Eigen::MatrixXd utils::symmetrize_covariance(const Eigen::MatrixXd &Sigma) {
  return 0.5*(Sigma + Sigma.transpose());
}


} // ns mmsf
