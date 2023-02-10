/******************************************************************************
* FILENAME:     IIKF_DICO.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Strategies/IIKF_DICO.hpp>
namespace ikf {

IIKF_DICO::IIKF_DICO(std::shared_ptr<IInstanceHandler> ptr_hdl, const std::string &name, size_t ID, const double horizon_sec) : IMMSF(ptr_hdl,name, ID, horizon_sec) {}

Eigen::MatrixXd IIKF_DICO::get_CrossCovFact_at_t(const Timestamp &t, size_t unique_ID) const{
  Eigen::MatrixXd mat;
  auto iter = HistCrossCovFactors.find(unique_ID);
  if (iter != HistCrossCovFactors.end()) {
    bool res = iter->second.get_at_t(t, mat);
    if (!res) {
      std::cout << "IMMSF.get_CrossCovFact_at_t(): could not find elem for id=" << unique_ID << " at t=" << t << std::endl;
    }
  }
  return mat;
}

Eigen::MatrixXd IIKF_DICO::get_CrossCovFact_before_t(const Timestamp &t, size_t unique_ID) const{
  Eigen::MatrixXd mat;
  auto iter = HistCrossCovFactors.find(unique_ID);
  if (iter != HistCrossCovFactors.end()) {
    bool res = iter->second.get_before_t(t, mat);
    if (!res) {
      std::cout << "IMMSF.get_CrossCovFact_at_t(): could not find elem for id=" << unique_ID << " at t=" << t << std::endl;
    }
  }
  return mat;
}

void IIKF_DICO::set_CrossCovFact_at_t(const Timestamp &t, const size_t unique_ID, const Eigen::MatrixXd &ccf) {
  auto iter = HistCrossCovFactors.find(unique_ID);
  if (iter != HistCrossCovFactors.end()) {
    iter->second.insert(ccf, t);
  } else {
    auto iter = HistCrossCovFactors.emplace(unique_ID, TTimeHorizonBuffer<Eigen::MatrixXd>(max_time_horizon_sec));
    iter.first->second.insert(ccf, t);
  }
}

void IIKF_DICO::propagate_CrossCovFact(const Timestamp &t_a, const Timestamp &t_b, const Eigen::MatrixXd &M_a_b) {
  for (auto &elem  : HistCrossCovFactors) {
    Eigen::MatrixXd CrossCovFact_ta;
    if (elem.second.get_at_t(t_a, CrossCovFact_ta)) {
      Eigen::MatrixXd CrossCovFact_tb = M_a_b * CrossCovFact_ta;
      elem.second.insert(CrossCovFact_tb, t_b);
    }
  }
}

void IIKF_DICO::set_crosscov(const std::shared_ptr<IIKF_DICO> &pIKF_I, const std::shared_ptr<IIKF_DICO> &pIKF_J, const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &Sigma_IJ, const Timestamp &t) {
  if (pIKF_I->m_ID < pIKF_J->m_ID) {
    pIKF_I->set_CrossCovFact_at_t(t, ID_J, Sigma_IJ);
    pIKF_J->set_CrossCovFact_at_t(t, ID_I, Eigen::MatrixXd::Identity(Sigma_IJ.cols(), Sigma_IJ.cols()));
  } else {
    Eigen::MatrixXd Sigma_JI = Sigma_IJ.transpose();
    pIKF_J->set_CrossCovFact_at_t(t, ID_I, Sigma_JI);
    pIKF_I->set_CrossCovFact_at_t(t, ID_J, Eigen::MatrixXd::Identity(Sigma_JI.cols(), Sigma_JI.cols()));
  }
}

Eigen::MatrixXd IIKF_DICO::get_crosscov(const std::shared_ptr<IIKF_DICO> &pIKF_I, const std::shared_ptr<IIKF_DICO> &pIKF_J, const size_t ID_I, const size_t ID_J, const Timestamp &t) {
  Eigen::MatrixXd  SigmaFact_IJ = pIKF_I->get_CrossCovFact_at_t(t, ID_J);
  Eigen::MatrixXd  SigmaFact_JI = pIKF_J->get_CrossCovFact_at_t(t, ID_I);

  Eigen::MatrixXd Sigma_IJ;
  if (SigmaFact_IJ.size() && SigmaFact_JI.size()) {
    if (pIKF_I->m_ID < pIKF_J->m_ID) {
      RTV_EXPECT_TRUE_MSG(SigmaFact_IJ.cols() == SigmaFact_JI.cols(), "get_crosscov wrong dims @t=" + t.str());
      Sigma_IJ = SigmaFact_IJ * SigmaFact_JI.transpose();

    } else {
      RTV_EXPECT_TRUE_MSG(SigmaFact_IJ.cols() == SigmaFact_JI.cols(), "get_crosscov wrong dims @t=" + t.str());
      Eigen::MatrixXd Sigma_JI = SigmaFact_JI * SigmaFact_IJ.transpose();
      Sigma_IJ =  Sigma_JI.transpose();
    }
  }
  return Sigma_IJ;
}

void IIKF_DICO::reset() {
  IMMSF::reset();
  HistCrossCovFactors.clear();
}

void IIKF_DICO::remove_beliefs_after_t(const Timestamp &t) {
  IMMSF::remove_beliefs_after_t(t);
  for (auto& elem : HistCrossCovFactors){
    elem.second.remove_after_t(t);
  }
}

void IIKF_DICO::set_horizon(const double t_hor) {
  IMMSF::set_horizon(t_hor);
  for (auto& elem : HistCrossCovFactors){
    elem.second.set_horizon(t_hor);
  }
}

void IIKF_DICO::check_horizon() {
  IMMSF::check_horizon();
  for (auto& elem : HistCrossCovFactors){
    elem.second.check_horizon();
  }
}


} // ns mmsf
