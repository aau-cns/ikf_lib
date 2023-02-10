/******************************************************************************
* FILENAME:     KalmanFilter.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     26.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP
#include <ikf/ikf_api.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <ikf/utils/eigen_utils.hpp>



namespace ikf {

class IKF_API KalmanFilter {
public:

  ///////////////////////////////////////////
  /// HELPER
  static Eigen::MatrixXd covariance_propagation(Eigen::MatrixXd const& Sigma_apri_a,
                                                Eigen::MatrixXd const& Phi_a_b,
                                                Eigen::MatrixXd const& Q_a,
                                                bool const nummerical_stabilization=false);


  struct CorrectionResult_t {
    Eigen::MatrixXd Sigma_apos;
    Eigen::VectorXd delta_mean; // dx
    Eigen::MatrixXd U; // U = (I-K*H)
    bool rejected = true;
  };
  struct CorrectionCfg_t {
    bool use_outlier_rejection = true;
    bool use_Josephs_form=true;
    bool nummerical_stabilization=true;
    double confidence_interval = 0.997;
    double eps = 1e-10;
  };

  static CorrectionResult_t correction_step(Eigen::MatrixXd const& H, Eigen::MatrixXd const& R,
                                            Eigen::VectorXd const & r, Eigen::MatrixXd const& Sigma_apri,
                                            CorrectionCfg_t const& cfg);




  static bool check_dim(Eigen::VectorXd const& mean, Eigen::MatrixXd const& Sigma);
  static bool check_dim(Eigen::MatrixXd const& Sigma_apri, Eigen::MatrixXd const& Phi, Eigen::MatrixXd const& Q);

  static bool check_dim(Eigen::MatrixXd const& H, Eigen::MatrixXd const& R, Eigen::VectorXd const& r,
                        Eigen::MatrixXd const& Sigma);

};


}
#endif // KALMANFILTER_HPP
