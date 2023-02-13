/******************************************************************************
* FILENAME:     IsolatedKalmanFilterStd.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/EstimatorStd/IsolatedKalmanFilterStd.hpp>
#include <ikf/EstimatorStd/IKFHandlerStd.hpp>
#include <ikf/Estimator/KalmanFilter.hpp>
#include <ikf/utils/RTVerification.hpp>
#include <ikf/utils/eigen_utils.hpp>

namespace ikf{

IsolatedKalmanFilterStd::IsolatedKalmanFilterStd(ptr_handler pHandler, const ptr_belief &belief, const size_t ID) : KalmanFilterStd(belief),  m_ID(ID), m_pHandler(pHandler) {}

IsolatedKalmanFilterStd::~IsolatedKalmanFilterStd() {}

size_t IsolatedKalmanFilterStd::ID() const { return m_ID; }

bool IsolatedKalmanFilterStd::propagate(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_b) {
  bool res = KalmanFilterStd::propagate(Phi_II_ab, Q_II_ab, t_b);
  if (res) {
    apply_propagation(Phi_II_ab);
  }
  return res;
}

bool IsolatedKalmanFilterStd::private_update(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R, const Eigen::VectorXd &z) {
  Eigen::MatrixXd Sigma_apri = m_belief->Sigma();
  bool res = KalmanFilterStd::private_update(H_II, R, z);
  if (res) {
    apply_correction(Sigma_apri, m_belief->Sigma());
  }
  return res;
}

bool IsolatedKalmanFilterStd::joint_update(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &R, const Eigen::VectorXd &z) {
  RTV_EXPECT_TRUE_THROW(m_pHandler->exists(ID_I) && m_pHandler->exists(ID_J), "IKF instances do not exists!");
  RTV_EXPECT_TRUE_THROW(ID_I == m_ID, "ID_I missmatch! wrong interim master");

  ptr_belief bel_J_apri = m_pHandler->get(ID_J)->get_belief();
  RTV_EXPECT_TRUE_THROW(bel_J_apri->timestamp() == m_belief->timestamp(), "Timestamps of bliefs need to match! Did you forgett to propagate first?");

  // stack the measurement sensitivity matrix:
  Eigen::MatrixXd H_joint = utils::horcat(H_II, H_JJ);
  // stack individual's covariances:
  Eigen::MatrixXd Sigma_joint = utils::stabilize_covariance(stack_apri_covariance(bel_J_apri, ID_J));
  // stack individual's mean:
  Eigen::VectorXd mean_joint = utils::vertcat_vec(m_belief->mean(), bel_J_apri->mean());

  // residual:
  Eigen::VectorXd r =  z - H_joint * mean_joint;

  KalmanFilter::CorrectionCfg_t cfg;
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_joint, R, r, Sigma_joint, cfg);
  if (!res.rejected) {
    RTV_EXPECT_TRUE_THROW(utils::is_positive_semidefinite(res.Sigma_apos), "Joint apos covariance is not PSD!");
    size_t dim_I = m_belief->Sigma().rows();
    size_t dim_J = bel_J_apri->Sigma().rows();

    // split covariance
    Eigen::MatrixXd Sigma_II_apos, Sigma_JJ_apos, Sigma_IJ_apos;
    split_Sigma(res.Sigma_apos, dim_I, dim_J, Sigma_II_apos, Sigma_JJ_apos, Sigma_IJ_apos);

    // IMPORTANT: keep order! before setting cross-covariance factors and beliefs implace!
    // 1) add correction terms in the appropriate correction buffers!
    apply_correction(m_belief->Sigma(), Sigma_II_apos);
    m_pHandler->get(ID_J)->apply_correction(bel_J_apri->Sigma(), Sigma_JJ_apos);

    // 2) set a corrected factorized a posterioiry cross-covariance
    set_Sigma_IJ(ID_I, ID_J, Sigma_IJ_apos);

    // 3) correct beliefs implace!
    m_belief->correct(res.delta_mean.topLeftCorner(m_belief->es_dim(), 1), Sigma_II_apos);
    bel_J_apri->correct(res.delta_mean.bottomRightCorner(bel_J_apri->es_dim(), 1), Sigma_JJ_apos);
  }
  return !res.rejected;
}

/////////////// PROTECTED
Eigen::MatrixXd IsolatedKalmanFilterStd::stack_Sigma(const Eigen::MatrixXd &Sigma_II, const Eigen::MatrixXd &Sigma_JJ, const Eigen::MatrixXd &Sigma_IJ) {

  RTV_EXPECT_TRUE_THROW(Sigma_IJ.size() != 0, "empty Sigma_IJ!");
  RTV_EXPECT_TRUE_THROW((Sigma_II.rows() == Sigma_IJ.rows()) && (Sigma_IJ.cols() == Sigma_JJ.cols()), "dimension missmatch!");

  Eigen::MatrixXd C(Sigma_II.rows()+Sigma_JJ.rows(), Sigma_II.cols() + Sigma_JJ.cols());

  C << Sigma_II, Sigma_IJ,
      Sigma_IJ.transpose(), Sigma_JJ;
  return C;
}

void IsolatedKalmanFilterStd::split_Sigma(const Eigen::MatrixXd &Sigma, const size_t dim_I, const size_t dim_J, Eigen::MatrixXd &Sigma_II, Eigen::MatrixXd &Sigma_JJ, Eigen::MatrixXd &Sigma_IJ) {
  RTV_EXPECT_TRUE_THROW(dim_I > 0 && dim_J > 0, "Dimension insvalid");

  RTV_EXPECT_TRUE_THROW((Sigma.rows() == (long)(dim_I + dim_J)) && (Sigma.cols() == (long)(dim_I + dim_J)), "dimension missmatch!");
  Sigma_II = Sigma.topLeftCorner(dim_I, dim_I);
  Sigma_JJ = Sigma.bottomRightCorner(dim_J, dim_J);
  Sigma_IJ = Sigma.topRightCorner(dim_I, dim_J);
}

void IsolatedKalmanFilterStd::set_CrossCovFact(const size_t ID_J, const Eigen::MatrixXd &SigmaFact_IJ) {
  m_CrossCovFactors[ID_J] =  SigmaFact_IJ;
}

void IsolatedKalmanFilterStd::set_Sigma_IJ(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &Sigma_IJ) {
  if (ID_I < ID_J) {
    m_pHandler->get(ID_I)->set_CrossCovFact(ID_J, Sigma_IJ);
    m_pHandler->get(ID_J)->set_CrossCovFact(ID_I, Eigen::MatrixXd::Identity(Sigma_IJ.cols(), Sigma_IJ.cols()));
  } else {
    Eigen::MatrixXd Sigma_JI = Sigma_IJ.transpose();
    m_pHandler->get(ID_J)->set_CrossCovFact(ID_I, Sigma_JI);
    m_pHandler->get(ID_I)->set_CrossCovFact(ID_J, Eigen::MatrixXd::Identity(Sigma_JI.cols(), Sigma_JI.cols()));
  }
}

Eigen::MatrixXd IsolatedKalmanFilterStd::get_CrossCovFact(const size_t ID_J) const {
  auto iter = m_CrossCovFactors.find(ID_J);
  if (iter != m_CrossCovFactors.end()) {
    return iter->second;
  }
  return Eigen::MatrixXd();
}

Eigen::MatrixXd IsolatedKalmanFilterStd::get_Sigma_IJ(const size_t ID_I, const size_t ID_J) {
  Eigen::MatrixXd  SigmaFact_IJ = m_pHandler->get(ID_I)->get_CrossCovFact(ID_J);
  Eigen::MatrixXd  SigmaFact_JI = m_pHandler->get(ID_J)->get_CrossCovFact(ID_I);
  Eigen::MatrixXd Sigma_IJ;
  if (SigmaFact_IJ.size() && SigmaFact_JI.size()) {
    if (ID_I < ID_J) {
      RTV_EXPECT_TRUE_MSG(SigmaFact_IJ.cols() == SigmaFact_JI.cols(), "get_crosscov wrong dims");
      Sigma_IJ = SigmaFact_IJ * SigmaFact_JI.transpose();

    } else {
      RTV_EXPECT_TRUE_MSG(SigmaFact_IJ.cols() == SigmaFact_JI.cols(), "get_crosscov wrong dims");
      Eigen::MatrixXd Sigma_JI = SigmaFact_JI * SigmaFact_IJ.transpose();
      Sigma_IJ =  Sigma_JI.transpose();
    }
  }
  return Sigma_IJ;
}


Eigen::MatrixXd IsolatedKalmanFilterStd::stack_apri_covariance(ptr_belief &bel_J_apri, const size_t ID_J) {
  Eigen::MatrixXd Sigma_IJ = get_Sigma_IJ(m_ID, ID_J);

  if (Sigma_IJ.size()) {
    return stack_Sigma(m_belief->Sigma(), bel_J_apri->Sigma(), Sigma_IJ);
  }
  else {
    // Not correlated!
    Sigma_IJ = Sigma_IJ.Zero(m_belief->Sigma().rows(), bel_J_apri->Sigma().cols());
    return stack_Sigma(m_belief->Sigma(), bel_J_apri->Sigma(), Sigma_IJ);
  }
}

void IsolatedKalmanFilterStd::apply_propagation(Eigen::MatrixXd const &Phi_a_b) {
  for (auto iter = m_CrossCovFactors.begin(); iter != m_CrossCovFactors.end(); iter++) {
    iter->second = Phi_a_b * (iter->second);
  }
}

void IsolatedKalmanFilterStd::apply_correction(Eigen::MatrixXd const &Lambda) {
  for (auto iter = m_CrossCovFactors.begin(); iter != m_CrossCovFactors.end(); iter++) {
    iter->second = Lambda * (iter->second);
  }
}

void IsolatedKalmanFilterStd::apply_correction(const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd &Sigma_apos) {
  // [1] EQ 26. Luft's approximation term/factor for non-participants' factorized cross-covariances.
  Eigen::MatrixXd Lambda = Sigma_apos * Sigma_apri.inverse();
  apply_correction(Lambda);
}


} // ns ikf
