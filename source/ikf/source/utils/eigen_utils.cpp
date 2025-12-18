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
  if (!H_II.size()) {
    return H_JJ;
  }
  if (!H_JJ.size()) {
    return H_II;
  }

  RTV_EXPECT_TRUE_THROW(H_II.rows() == H_JJ.rows(), "dimension missmatch!");

  Eigen::MatrixXd C(H_II.rows(), H_II.cols()+H_JJ.cols());
  C.leftCols(H_II.cols()) = H_II;
  C.rightCols(H_JJ.cols()) = H_JJ;
  return C;
}

Eigen::MatrixXd utils::horcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &H_KK) {
  RTV_EXPECT_TRUE_THROW(H_II.rows() == H_JJ.rows() && H_II.rows() == H_KK.rows() , "dimension missmatch!");

  Eigen::MatrixXd C(H_II.rows(), H_II.cols()+H_JJ.cols()+H_KK.cols());
  C.leftCols(H_II.cols()) = H_II;
  C.middleCols(H_II.cols(), H_JJ.cols()) = H_JJ;
  C.rightCols(H_KK.cols()) = H_KK;
  return C;
}


Eigen::MatrixXd ikf::utils::horcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &H_KK, const Eigen::MatrixXd &H_LL) {
  RTV_EXPECT_TRUE_THROW(H_II.rows() == H_JJ.rows() && H_II.rows() == H_KK.rows() && H_II.rows() == H_LL.rows() , "dimension missmatch!");

  Eigen::MatrixXd C(H_II.rows(), H_II.cols()+H_JJ.cols()+H_KK.cols()+H_LL.cols());
  C.leftCols(H_II.cols()) = H_II;
  C.middleCols(H_II.cols(), H_JJ.cols()) = H_JJ;
  C.middleCols(H_II.cols()+H_JJ.cols(), H_KK.cols()) = H_KK;
  C.rightCols(H_LL.cols()) = H_LL;
  return C;
}


Eigen::MatrixXd utils::vertcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ) {
  if (!H_II.size()) {
    return H_JJ;
  }
  if (!H_JJ.size()) {
    return H_II;
  }

  RTV_EXPECT_TRUE_THROW(H_II.cols() == H_JJ.cols(), "dimension missmatch!");

  Eigen::MatrixXd C(H_II.rows()+H_JJ.rows(), H_JJ.cols());
  C << H_II,
      H_JJ;
  return C;
}

Eigen::MatrixXd ikf::utils::vertcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &H_KK) {
  RTV_EXPECT_TRUE_THROW(H_II.cols() == H_JJ.cols() && H_II.cols() == H_KK.cols(), "dimension missmatch!");

  Eigen::MatrixXd C(H_II.rows()+H_JJ.rows()+H_KK.rows(), H_JJ.cols());
  C << H_II,
      H_JJ,
      H_KK;
  return C;
}

Eigen::MatrixXd ikf::utils::vertcat(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &H_KK, const Eigen::MatrixXd &H_LL) {
  RTV_EXPECT_TRUE_THROW(H_II.cols() == H_JJ.cols() && H_II.cols() == H_KK.cols() && H_II.cols() == H_LL.cols(), "dimension missmatch!");

  Eigen::MatrixXd C(H_II.rows()+H_JJ.rows()+H_KK.rows()+H_LL.rows(), H_JJ.cols());
  C << H_II,
      H_JJ,
      H_KK,
      H_LL;
  return C;
}


Eigen::VectorXd utils::vertcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ) {
  if (!v_II.size()) {
    return v_JJ;
  }
  if (!v_JJ.size()) {
    return v_II;
  }

  RTV_EXPECT_TRUE_THROW(v_II.cols() == v_JJ.cols(), "dimension missmatch!");

  Eigen::VectorXd C(v_II.rows()+v_JJ.rows(), v_JJ.cols());
  C << v_II,
      v_JJ;
  return C;
}

Eigen::VectorXd utils::vertcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ, const Eigen::VectorXd &v_KK) {
  RTV_EXPECT_TRUE_THROW(v_II.cols() == v_JJ.cols(), "dimension missmatch!");
  RTV_EXPECT_TRUE_THROW(v_II.cols() == v_KK.cols(), "dimension missmatch!");
  Eigen::VectorXd C(v_II.rows()+v_JJ.rows()+v_KK.rows(), v_JJ.cols());
  C << v_II, v_JJ, v_KK;
  return C;

}
Eigen::VectorXd utils::horcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ) {
  if (!v_II.size()) {
    return v_JJ;
  }
  if (!v_JJ.size()) {
    return v_II;
  }

  RTV_EXPECT_TRUE_THROW(v_II.rows() == v_JJ.rows(), "dimension missmatch!");

  Eigen::VectorXd C(v_II.rows(), v_II.cols()+v_JJ.cols());
  C << v_II,
      v_JJ;
  return C;
}

Eigen::VectorXd utils::horcat_vec(const Eigen::VectorXd &v_II, const Eigen::VectorXd &v_JJ, const Eigen::VectorXd &v_KK) {
  RTV_EXPECT_TRUE_THROW(v_II.rows() == v_JJ.rows(), "dimension missmatch!");
  RTV_EXPECT_TRUE_THROW(v_II.rows() == v_KK.rows(), "dimension missmatch!");

  Eigen::VectorXd C(v_II.rows(), v_II.cols()+v_JJ.cols()+v_KK.cols());
  C << v_II, v_JJ, v_KK;
  return C;
}


Eigen::MatrixXd utils::stabilize_covariance(const Eigen::MatrixXd &Sigma, const double eps) {
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Sigma.rows(), Sigma.cols())*eps;
  return 0.5*(Sigma + Sigma.transpose()) + I;
}

Eigen::MatrixXd utils::symmetrize_covariance(const Eigen::MatrixXd &Sigma) {
  return 0.5*(Sigma + Sigma.transpose());
}

void utils::split_Sigma(const Eigen::MatrixXd &Sigma, const size_t dim_I, const size_t dim_J, const size_t dim_K,
                        const size_t dim_L, Eigen::MatrixXd &Sigma_II, Eigen::MatrixXd &Sigma_JJ,
                        Eigen::MatrixXd &Sigma_KK, Eigen::MatrixXd &Sigma_LL, Eigen::MatrixXd &Sigma_IJ,
                        Eigen::MatrixXd &Sigma_IK, Eigen::MatrixXd &Sigma_JK, Eigen::MatrixXd &Sigma_IL,
                        Eigen::MatrixXd &Sigma_JL, Eigen::MatrixXd &Sigma_KL) {
  Sigma_II = Sigma.topLeftCorner(dim_I, dim_I);
  Sigma_JJ = Sigma.block(dim_I, dim_I, dim_J, dim_J);
  Sigma_KK = Sigma.block(dim_I + dim_J, dim_I + dim_J, dim_K, dim_K);
  Sigma_LL = Sigma.bottomRightCorner(dim_L, dim_L);

  Sigma_IJ = Sigma.block(0, dim_I, dim_I, dim_J);
  Sigma_IK = Sigma.block(0, dim_I + dim_J, dim_I, dim_K);
  Sigma_IL = Sigma.topRightCorner(dim_I, dim_L);

  Sigma_JK = Sigma.block(dim_I, dim_I + dim_J, dim_J, dim_K);
  Sigma_JL = Sigma.block(dim_I, dim_I + dim_J + dim_K, dim_J, dim_L);

  Sigma_KL = Sigma.block(dim_I + dim_J, dim_I + dim_J + dim_K, dim_K, dim_L);
}

void utils::split_Sigma(const Eigen::MatrixXd &Sigma, const size_t dim_I, const size_t dim_J, const size_t dim_K,
                        Eigen::MatrixXd &Sigma_II, Eigen::MatrixXd &Sigma_JJ, Eigen::MatrixXd &Sigma_KK,
                        Eigen::MatrixXd &Sigma_IJ, Eigen::MatrixXd &Sigma_IK, Eigen::MatrixXd &Sigma_JK) {
  RTV_EXPECT_TRUE_THROW(dim_I > 0 && dim_J > 0 && dim_K > 0, "Dimension insvalid");
  RTV_EXPECT_TRUE_THROW(
    (Sigma.rows() == (long)(dim_I + dim_J + dim_K)) && (Sigma.cols() == (long)(dim_I + dim_J + dim_K)),
    "dimension missmatch!");

  Sigma_II = Sigma.topLeftCorner(dim_I, dim_I);
  Sigma_JJ = Sigma.block(dim_I, dim_I, dim_J, dim_J);
  Sigma_IJ = Sigma.block(0, dim_I, dim_I, dim_J);
  Sigma_IK = Sigma.topRightCorner(dim_I, dim_K);
  Sigma_JK = Sigma.block(dim_I, dim_I + dim_J, dim_J, dim_K);
  Sigma_KK = Sigma.bottomRightCorner(dim_K, dim_K);
}

void utils::split_Sigma(const Eigen::MatrixXd &Sigma, const size_t dim_I, const size_t dim_J, Eigen::MatrixXd &Sigma_II,
                        Eigen::MatrixXd &Sigma_JJ, Eigen::MatrixXd &Sigma_IJ) {
  RTV_EXPECT_TRUE_THROW(dim_I > 0 && dim_J > 0, "Dimension insvalid");

  RTV_EXPECT_TRUE_THROW((Sigma.rows() == (long)(dim_I + dim_J)) && (Sigma.cols() == (long)(dim_I + dim_J)),
                        "dimension missmatch!");
  Sigma_II = Sigma.topLeftCorner(dim_I, dim_I);
  Sigma_JJ = Sigma.bottomRightCorner(dim_J, dim_J);
  Sigma_IJ = Sigma.topRightCorner(dim_I, dim_J);
}

Eigen::MatrixXd utils::stack_Sigma(const Eigen::MatrixXd &Sigma_II, const Eigen::MatrixXd &Sigma_JJ,
                                   const Eigen::MatrixXd &Sigma_IJ) {
  RTV_EXPECT_TRUE_THROW(Sigma_IJ.size() != 0, "empty Sigma_IJ!");
  RTV_EXPECT_TRUE_THROW((Sigma_II.rows() == Sigma_IJ.rows()) && (Sigma_IJ.cols() == Sigma_JJ.cols()),
                        "dimension missmatch!");

  Eigen::MatrixXd C(Sigma_II.rows() + Sigma_JJ.rows(), Sigma_II.cols() + Sigma_JJ.cols());

  C << Sigma_II, Sigma_IJ, Sigma_IJ.transpose(), Sigma_JJ;
  return C;
}

Eigen::MatrixXd utils::nearest_covariance(const Eigen::MatrixXd &Sigma, const double eps) {
  // Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
  //
  // All rights reserved.
  // https://github.com/aau-cns/mars_lib/blob/main/source/mars/source/nearest_cov.cpp
  //

  Eigen::EigenSolver<Eigen::MatrixXd> SVD(symmetrize_covariance(Sigma));

  Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType V(SVD.eigenvectors());
  Eigen::EigenSolver<Eigen::MatrixXd>::EigenvalueType D(SVD.eigenvalues());

  Eigen::MatrixXd V_real(V.real());
  Eigen::VectorXd D_real(D.real());

  // determine if the matrix is already positive-semi-definite
  bool negative_eigenvalues = false;
  for (int k = 0; k < D_real.size(); k++) {
    if (D_real[k] < 0) {
      negative_eigenvalues = true;
    }
  }

  if (!negative_eigenvalues) {
    return Sigma;
  }

  Eigen::VectorXd D_corrected(D_real);
  for (int k = 0; k < D_corrected.size(); k++) {
    if (D_corrected[k] < 0) {
      D_corrected[k] = eps;
    }
  }

  Eigen::MatrixXd result(V_real * D_corrected.asDiagonal() * V_real.inverse());
  return result;
}

bool utils::correct_covariance(Eigen::MatrixXd &Sigma) {
  Sigma = utils::stabilize_covariance(Sigma);
  bool is_psd = utils::is_positive_semidefinite(Sigma);
  RTV_EXPECT_TRUE_MSG(is_psd, "utils::correct_covariance(): covariance is not PSD");
  if (!is_psd) {
    Sigma = utils::nearest_covariance(Sigma, 1e-6);
    return utils::is_positive_semidefinite(Sigma);
  }
  return true;
}

Eigen::MatrixXd utils::blkdiag(const Eigen::MatrixXd &Sigma_II, const Eigen::MatrixXd &Sigma_JJ) {
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(Sigma_II.rows() + Sigma_JJ.rows(), Sigma_II.cols() + Sigma_JJ.cols());
  C.block(0,0, Sigma_II.rows(), Sigma_JJ.cols()) = Sigma_II;
  C.block(Sigma_II.rows(), Sigma_JJ.cols(), Sigma_JJ.rows(), Sigma_JJ.cols()) = Sigma_JJ;
  //Eigen::MatrixXd Sigma_IJ = Eigen::MatrixXd::Zero(Sigma_II.rows(), Sigma_JJ.cols());
  //C << Sigma_II, Sigma_IJ, Sigma_IJ.transpose(), Sigma_JJ;
  return C;
}

}  // namespace ikf
