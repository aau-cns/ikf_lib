/******************************************************************************
* FILENAME:     MMSF_IKF.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Strategies/MMSF_IKF.hpp>
#include <ikf/utils/RTVerification.hpp>

namespace ikf {

MMSF_IKF::MMSF_IKF(std::shared_ptr<IInstanceHandler> ptr_hdl, const std::string &name, size_t ID, const double horizon_sec) : IIKF_DICO(ptr_hdl,name, ID, horizon_sec), HistCorr(horizon_sec) {
}

void MMSF_IKF::reset() {
  IIKF_DICO::reset();
  HistCorr.clear();
}

void MMSF_IKF::remove_beliefs_after_t(const Timestamp &t) {
  IIKF_DICO::remove_beliefs_after_t(t);
  HistCorr.remove_after_t(t);
}

void MMSF_IKF::set_horizon(const double t_hor) {
  IIKF_DICO::set_horizon(t_hor);
  HistCorr.set_horizon(t_hor);
}

void MMSF_IKF::check_horizon() {
  IIKF_DICO::check_horizon();
  HistCorr.check_horizon();
}

Eigen::MatrixXd MMSF_IKF::compute_correction(const Timestamp &t_a, const Timestamp &t_b) {
  TStampedData<Eigen::MatrixXd> data_ta, data_tb;

  bool exist_after_ta = HistCorr.get_after_t(t_a, data_ta);
  bool exist_at_tb = HistCorr.get_at_t(t_b, data_tb);

  Timestamp t_b_found = t_b;

  if (!exist_at_tb) {
    exist_at_tb= HistCorr.get_before_t(t_b, data_tb);
    t_b_found = data_tb.stamp;
  }

  if (exist_after_ta && exist_at_tb) {
    int max_dim = std::max(data_ta.data.cols(), data_ta.data.rows());
    Eigen::MatrixXd I  = Eigen::MatrixXd::Identity(max_dim, max_dim);
    Eigen::MatrixXd M_a_b = HistCorr.accumulate_between_t1_t2(t_a, t_b_found, I,
                                                              [](Eigen::MatrixXd const& A, Eigen::MatrixXd const& B){
                                                                Eigen::MatrixXd C(A.rows(), A.cols());
                                                                C = B*A;
                                                                return C;
                                                              });
    return M_a_b;
  }
  else {
    std::cout << "IMMF::compute_correction(): no element found for timestamps:[" << t_a << "," << t_b_found << "]" << std::endl;
  }
  return Eigen::MatrixXd();
}

bool MMSF_IKF::add_correction_at_t(const Timestamp &t_b, const Eigen::MatrixXd &Phi_a_b) {
  if(!HistCorr.exist_at_t(t_b)) {
    HistCorr.insert(Phi_a_b, t_b);
    return true;
  }
  else {
    // This case should not happen, but if it does, we can handle the situations:

    std::cout << "IMMF::add_correction_at_t(): correction term already exists at t_b:[" << t_b << "]" << std::endl;
    Eigen::MatrixXd Factor_b;
    if (HistCorr.get_at_t(t_b, Factor_b)) {
      std::cout << "IMMF::add_correction_at_t(): right multiply the propagation factor at t_b:[" << t_b << "]" << std::endl;
      Factor_b = Factor_b*Phi_a_b;
      HistCorr.insert(Factor_b, t_b);

      // update all cross-covariance that already exist at timestamp t_a:
      // Otherwise they would never exerperience it: (special case if many
      // observations happen at the same timestep. The first cross-covariance would not receive corrections in the next time steps, just the last one would be exact!)
      for (auto & hist : HistCrossCovFactors) {
        Eigen::MatrixXd Sigma_12_b;
        if (hist.second.get_at_t(t_b, Sigma_12_b)) {
          Sigma_12_b = Sigma_12_b * Phi_a_b;
          hist.second.insert(Sigma_12_b, t_b);
        }
      }

      return true;
    }
    return false;
  }
}

bool MMSF_IKF::apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Factor){
  RTV_EXPECT_TRUE_MSG(Factor.cols() == Factor.rows(), "Factor must be a square matrix!");
  Eigen::MatrixXd mat;
  if (HistCorr.get_at_t(t, mat)) {
    mat = Factor * mat;
    HistCorr.insert(mat, t);
    return true;
  }
  else {
    HistCorr.insert(Factor, t);
    std::cout << "MMSF_IKF::apply_correction_at_t(): no element found for timestamps:[" << t << "]" << std::endl;
    return false;
  }
}

bool MMSF_IKF::apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd Sigma_apos){
  Eigen::MatrixXd Lambda = Sigma_apos * Sigma_apri.inverse();
  return apply_correction_at_t(t, Lambda);
}

bool MMSF_IKF::apply_propagation(const size_t ID_I, const Eigen::VectorXd &mean_II, const Eigen::MatrixXd &Phi_II,
                                 const Eigen::MatrixXd &Q_II, const Timestamp &t_old, const Timestamp &t_new) {
  return true;
}

bool MMSF_IKF::apply_private_observation(const size_t ID_I, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                         const Eigen::VectorXd &r, const Timestamp &t) {
  return true;
}

bool MMSF_IKF::apply_joint_observation(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
                                       const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t) {
  return true;
}



} // namespace ikf
