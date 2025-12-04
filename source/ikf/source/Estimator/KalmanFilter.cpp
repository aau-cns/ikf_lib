/******************************************************************************
* FILENAME:     KalmanFilter.cpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
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
#include <ikf/Estimator/KalmanFilter.hpp>
#include <ikf/Estimator/NormalizedInnovationSquared.hpp>
#include <ikf/utils/eigen_utils.hpp>
#include <ikf/utils/eigen_utils.hpp>
#include <ikf/Logger/Logger.hpp>
namespace ikf {

Eigen::MatrixXd KalmanFilter::covariance_propagation(const Eigen::MatrixXd &Sigma_apri_a, const Eigen::MatrixXd &Phi_a_b, const Eigen::MatrixXd &Q_a, const bool nummerical_stabilization) {
  Eigen::MatrixXd Sigma_apri_b;
  if (check_dim(Sigma_apri_a, Phi_a_b, Q_a)) {
    if (nummerical_stabilization) {
      Sigma_apri_b = Phi_a_b * utils::stabilize_covariance(Sigma_apri_a) * Phi_a_b.transpose() + utils::stabilize_covariance(Q_a);
      Sigma_apri_b = utils::stabilize_covariance(Sigma_apri_b);
    }
    else {
      Sigma_apri_b = Phi_a_b * Sigma_apri_a * Phi_a_b.transpose() + Q_a;
    }
  }
  return Sigma_apri_b;
}

KalmanFilter::CorrectionResult_t KalmanFilter::correction_step(const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Eigen::MatrixXd &Sigma_apri, const CorrectionCfg_t &cfg) {
  CorrectionResult_t res;

  if (check_dim(H, R, r, Sigma_apri)) {
    Eigen::Index const dim = Sigma_apri.rows();
    res.rejected = false;

    // innovation covariance:
    Eigen::MatrixXd S = H * Sigma_apri  * H.transpose() + R;
    S = utils::stabilize_covariance(S, cfg.eps);
    if (cfg.use_outlier_rejection) {
      if (!NormalizedInnovationSquared::check_NIS(S, r, cfg.confidence_interval)) {
        res.rejected = true;
      }
    }

    // Kalman gain:
    Eigen::MatrixXd K = Sigma_apri*H.transpose() * S.inverse();
    res.delta_mean = K * r;
    res.U = (Eigen::MatrixXd::Identity(dim, dim) - K*H);

    if (cfg.use_Josephs_form) {
      res.Sigma_apos =  res.U * Sigma_apri * res.U.transpose() + K * R * K.transpose();
    }
    else {
      res.Sigma_apos = res.U * Sigma_apri;
    }

    if (cfg.nummerical_stabilization) {
      res.Sigma_apos = utils::stabilize_covariance(res.Sigma_apos, cfg.eps);
    }
    res.Sigma_apos = utils::symmetrize_covariance(res.Sigma_apos);
  }
  return res;
}

bool KalmanFilter::check_dim(const Eigen::VectorXd &mean, const Eigen::MatrixXd &Sigma) {
  if ( (mean.cols() != 1) || (mean.rows() != Sigma.rows()) || (Sigma.rows() != Sigma.cols()) ) {
    Logger::ikf_logger()->error("KalmanFilter::check_dim(): dimensionality miss match of mean " + utils::get_shape(mean) + " and the Sigma" + utils::get_shape(Sigma));
    return false;
  }
  return true;
}

bool ikf::KalmanFilter::check_dim(const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd &Phi) {
  bool good = true;
  if ((Sigma_apri.rows() != Phi.rows()) || (Sigma_apri.rows() != Phi.cols())) {
    Logger::ikf_logger()->info("KalmanFilter::check_dim(): dimensionality miss match of Sigma_apri " + utils::get_shape(Sigma_apri) + " and the Phi" + utils::get_shape(Phi));
    good = false;
  }


  return good;
}

bool KalmanFilter::check_dim(const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &Q) {
  bool good = check_dim(Sigma_apri, Phi);

  if ((Sigma_apri.rows() != Q.rows()) || (Sigma_apri.rows() != Q.cols())) {
    Logger::ikf_logger()->info("KalmanFilter::check_dim(): dimensionality miss match of Sigma_apri " + utils::get_shape(Sigma_apri) + " and the Q" + utils::get_shape(Q));
    good = false;
  }
  return good;
}

bool KalmanFilter::check_dim(const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Eigen::MatrixXd &Sigma) {
  bool good = true;
  if (H.cols() != Sigma.rows()) {
    Logger::ikf_logger()->info("KalmanFilter::check_dim(): dimensionality miss match of H" + utils::get_shape(H) + "and the covaraince Sigma " + utils::get_shape(Sigma));
    good = false;
  }
  if (H.rows() != r.rows()) {
    Logger::ikf_logger()->info("KalmanFilter::check_dim(): dimensionality miss match of H" + utils::get_shape(H) + "and the residual/innovation r " + utils::get_shape(r));
    good = false;
  }
  if ((r.cols() != 1) || (r.rows() != H.rows())) {
    Logger::ikf_logger()->info("KalmanFilter::check_dim(): residual must be a colum vector! r" + utils::get_shape(r));
    good = false;
  }
  if ((H.rows() != R.rows()) || (H.rows() != R.cols())) {
    Logger::ikf_logger()->info("KalmanFilter::check_dim(): dimensionality miss match of H " + utils::get_shape(H) + " and the R" + utils::get_shape(R));
    good = false;
  }
  return good;
}

ikf::KalmanFilter::CorrectionResult_t ikf::KalmanFilter::covariance_intersection_correction(
  const Eigen::MatrixXd &H_ii, const Eigen::MatrixXd &H_jj, const Eigen::MatrixXd &R, const Eigen::VectorXd &r,
  const Eigen::MatrixXd &Sigma_ii_apri, const Eigen::MatrixXd &Sigma_jj_apri, const double omega_i,
  const CorrectionCfg_t &cfg) {
  CorrectionResult_t res;

  if (check_dim(H_ii, R, r, Sigma_ii_apri) && check_dim(H_jj, R, r, Sigma_jj_apri)) {
    res.rejected = false;

    // constant: Sec 5 [a]
    double omega_j = 1.0 - omega_i;

    //  S: Eq (29) [a] (innovation from CI)
    Eigen::MatrixXd S = (1 / omega_j) * H_jj * Sigma_jj_apri * H_jj.transpose() + R;

    // outlier rejection:
    S = utils::stabilize_covariance(S, cfg.eps);
    if (cfg.use_outlier_rejection) {
      if (!NormalizedInnovationSquared::check_NIS(S, r, cfg.confidence_interval)) {
        res.rejected = true;
      }
    }
    Eigen::MatrixXd S_inv = S.inverse();
    // Sigma_ii_apos: Eq (28) [a]
    res.Sigma_apos = (1 / omega_i) * Sigma_ii_apri
                     - (1 / std::pow(omega_i, 2)) * Sigma_ii_apri * H_ii.transpose() * S_inv * H_ii * Sigma_ii_apri;

    // delta_x: Eq (27) [a]
    res.delta_mean = (1 / omega_i) * Sigma_ii_apri * H_ii.transpose() * S_inv * r;

    if (cfg.nummerical_stabilization) {
      res.Sigma_apos = utils::stabilize_covariance(res.Sigma_apos, cfg.eps);
    }
  }

  return res;
}

} // ns mmsf
