/******************************************************************************
* FILENAME:     KalmanFilter.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Sensor/Estimator/KalmanFilter.hpp>
#include <ikf/Sensor/Estimator/NormalizedInnovationSquared.hpp>

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
    size_t const dim = Sigma_apri.rows();
    res.rejected = false;


    Eigen::MatrixXd S = H * Sigma_apri  * H.transpose() + R;
    S = utils::stabilize_covariance(S, cfg.eps);
    if (cfg.use_outlier_rejection) {
      if (!NormalizedInnovationSquared::check_NIS(S, r, cfg.confidence_interval)) {
        res.rejected = true;
      }
      else {
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
      }
    }
  }
  return res;
}

bool KalmanFilter::check_dim(const Eigen::VectorXd &mean, const Eigen::MatrixXd &Sigma) {
  if ( (mean.cols() != 1) || (mean.rows() != Sigma.rows()) || (Sigma.rows() != Sigma.cols()) ) {
    std::cout << "IKalmanFilter::check_dim(): dimensionality miss match of mean and Sigma " << utils::get_shape(mean) << " and the Q" << utils::get_shape(Sigma) << std::endl;
    return false;
  }
  return true;
}

bool KalmanFilter::check_dim(const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &Q) {
  bool good = true;

  if ((Sigma_apri.rows() != Q.rows()) || (Sigma_apri.rows() != Q.cols())) {
    std::cout << "IKalmanFilter::check_dim(): dimensionality miss match of Sigma_apri " << utils::get_shape(Sigma_apri) << " and the Q" << utils::get_shape(Q) << std::endl;
    good = false;
  }
  if ((Sigma_apri.rows() != Phi.rows()) || (Sigma_apri.rows() != Phi.cols())) {
    std::cout << "IKalmanFilter::check_dim(): dimensionality miss match of Sigma_apri " << utils::get_shape(Sigma_apri) << " and the Phi" << utils::get_shape(Phi) << std::endl;
    good = false;
  }
  return good;
}

bool KalmanFilter::check_dim(const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Eigen::MatrixXd &Sigma) {
  bool good = true;
  if (H.cols() != Sigma.rows()) {
    std::cout << "IKalmanFilter::check_dim(): dimensionality miss match of H"<< utils::get_shape(H) << "and the covaraince Sigma " << utils::get_shape(Sigma) << std::endl;
    good = false;
  }
  if (H.rows() != r.rows()) {
    std::cout << "IKalmanFilter::check_dim(): dimensionality miss match of H"<< utils::get_shape(H) << "and the residual/innovation r " << utils::get_shape(r) << std::endl;
    good = false;
  }
  if ((r.cols() != 1) || (r.rows() != H.rows())) {
    std::cout << "IKalmanFilter::check_dim(): residual must be a colum vector! r"<< utils::get_shape(r) << std::endl;
    good = false;
  }
  if ((H.rows() != R.rows()) || (H.rows() != R.cols())) {
    std::cout << "IKalmanFilter::check_dim(): dimensionality miss match of H " << utils::get_shape(H) << " and the R" << utils::get_shape(R) << std::endl;
    good = false;
  }
  return good;
}

} // ns mmsf
