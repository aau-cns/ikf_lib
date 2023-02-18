/******************************************************************************
* FILENAME:     IsolatedKalmanFilter.cpp
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
*
*  References:
*  [1] Jung, Roland and Weiss, Stephan, "Modular Multi-Sensor Fusion: A Collaborative State Estimation Perspective", IEEE RA-L, DOI: 10.1109/LRA.2021.3096165, 2021.
******************************************************************************/
#include <ikf/Estimator/IIsolatedKalmanFilter.hpp>
#include <ikf/Estimator/IsolatedKalmanFilterHandler.hpp>
#include <ikf/Estimator/KalmanFilter.hpp>
#include <ikf/utils/eigen_utils.hpp>


namespace ikf {

IIsolatedKalmanFilter::IIsolatedKalmanFilter(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, const size_t ID, const bool handle_delayed_meas, const double horizon_sec) : IKalmanFilter(horizon_sec, handle_delayed_meas), ptr_Handler(ptr_Handler), m_ID(ID) {

}

size_t IIsolatedKalmanFilter::ID() const { return m_ID; }


void IIsolatedKalmanFilter::reset() {
  IKalmanFilter::reset();
  HistCrossCovFactors.clear();
}

// Algorithm 7 in [1]
ProcessMeasResult_t IIsolatedKalmanFilter::process_measurement(const MeasData &m) {
  // propagation and private -> to filter instance
  // joint -> use ptr_Handler and two filter instances: The CIH must provide others belief and ccf
  ProcessMeasResult_t res = reprocess_measurement(m);

  if (m_handle_delayed_meas) {
    if (!res.rejected && m.obs_type != eObservationType::PROPAGATION) {

      TMultiHistoryBuffer<MeasData> meas_data = ptr_Handler->get_measurements_after_t(m.t_m);
      if(meas_data.exist_after_t(m.t_m)) {
        this->ptr_Handler->remove_beliefs_after_t(m.t_m);
        meas_data.foreach([this](MeasData const& m) {
          this->ptr_Handler->reprocess_measurement(m);
        });
      }
    }

    HistMeas.insert(m, m.t_m);
  }


  return res;
}

void IIsolatedKalmanFilter::initialize(ptr_belief bel_init) {
  initialize(bel_init, bel_init->timestamp());
}

void IIsolatedKalmanFilter::initialize(ptr_belief bel_init, const Timestamp &t) {
  reset();
  HistBelief.insert(bel_init, t);
}

bool IIsolatedKalmanFilter::redo_updates_after_t(const Timestamp &t) {
  remove_after_t(t);
  return IKalmanFilter::redo_updates_after_t(t);
}



bool IIsolatedKalmanFilter::get_CrossCovFact_at_t(const Timestamp &t, size_t ID_J, Eigen::MatrixXd &FFC){
  FFC = get_CrossCovFact_at_t(t, ID_J);
  return FFC.size() > 0;
}

Eigen::MatrixXd IIsolatedKalmanFilter::get_CrossCovFact_at_t(const Timestamp &t, size_t ID_J) {
  Eigen::MatrixXd mat;
  auto iter = HistCrossCovFactors.find(ID_J);
  if (iter != HistCrossCovFactors.end()) {
    iter->second.get_at_t(t, mat);
  }
  return mat;
}

Eigen::MatrixXd IIsolatedKalmanFilter::get_CrossCovFact_before_t(const Timestamp &t, size_t unique_ID) const{
  Eigen::MatrixXd mat;
  auto iter = HistCrossCovFactors.find(unique_ID);
  if (iter != HistCrossCovFactors.end()) {
    bool res = iter->second.get_before_t(t, mat);
    if (!res) {
      std::cout << "IMMSF.get_CrossCovFact_before_t(): could not find elem for id=" << unique_ID << " at t=" << t << std::endl;
    }
  }
  return mat;
}

void IIsolatedKalmanFilter::set_CrossCovFact_at_t(const Timestamp &t, const size_t unique_ID, const Eigen::MatrixXd &ccf) {
  auto iter = HistCrossCovFactors.find(unique_ID);
  if (iter != HistCrossCovFactors.end()) {
    iter->second.insert(ccf, t);
  } else {
    auto iter = HistCrossCovFactors.emplace(unique_ID, TTimeHorizonBuffer<Eigen::MatrixXd>(max_time_horizon_sec));
    iter.first->second.insert(ccf, t);
  }
}

void IIsolatedKalmanFilter::propagate_CrossCovFact(const Timestamp &t_a, const Timestamp &t_b, const Eigen::MatrixXd &M_a_b) {
  for (auto &elem  : HistCrossCovFactors) {
    Eigen::MatrixXd CrossCovFact_ta;
    if (elem.second.get_at_t(t_a, CrossCovFact_ta)) {
      Eigen::MatrixXd CrossCovFact_tb = M_a_b * CrossCovFact_ta;
      elem.second.insert(CrossCovFact_tb, t_b);
    }
  }
}


void IIsolatedKalmanFilter::remove_after_t(const Timestamp &t) {
  IKalmanFilter::remove_beliefs_after_t(t);
  for (auto& elem : HistCrossCovFactors){
    elem.second.remove_after_t(t);
  }
}

void IIsolatedKalmanFilter::remove_from_t(const Timestamp &t) {
  HistBelief.remove_after_t(t);
  HistBelief.remove_at_t(t);
  for (auto& elem : HistCrossCovFactors){
    elem.second.remove_after_t(t);
    elem.second.remove_at_t(t);
  }
}

void IIsolatedKalmanFilter::set_horizon(const double t_hor) {
  IKalmanFilter::set_horizon(t_hor*2);
  // measurement horizon can be max half the horizon of beliefs!
  HistMeas.set_horizon(t_hor);
  for (auto& elem : HistCrossCovFactors){
    elem.second.set_horizon(t_hor*2);
  }
}

void IIsolatedKalmanFilter::check_horizon() {
  IKalmanFilter::check_horizon();

  for (auto& elem : HistCrossCovFactors){
    elem.second.check_horizon();
  }
}


bool IIsolatedKalmanFilter::add_correction_at_t(const Timestamp &t_a, const Timestamp &t_b, const Eigen::MatrixXd &Phi_a_b) {
  // apply correction to exisit cross-covariance factors at t
  for(auto & HistCCF : HistCrossCovFactors) {
    if(HistCCF.second.exist_at_t(t_a))
    {
      Eigen::MatrixXd ccf_IJ_t;
      HistCCF.second.get_at_t(t_a, ccf_IJ_t);
      HistCCF.second.insert(Phi_a_b*ccf_IJ_t, t_b);
    }
  }
  return true;
}

bool IIsolatedKalmanFilter::apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Factor){
  // apply correction to exisit cross-covariance factors at t
  for(auto & HistCCF : HistCrossCovFactors) {
    Timestamp latest_ccf_t;
    if(HistCCF.second.exist_at_t(t)) {
      Eigen::MatrixXd ccf_IJ_t;
      HistCCF.second.get_at_t(t, ccf_IJ_t);
      HistCCF.second.insert(Factor*ccf_IJ_t, t);
    }
  }
  return true;
}

// Eq. 20 in [1]
bool IIsolatedKalmanFilter::apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd Sigma_apos){
  Eigen::MatrixXd Lambda = Sigma_apos * Sigma_apri.inverse();
  return apply_correction_at_t(t, Lambda);
}


// Algorithm 7 in [1]
ProcessMeasResult_t IIsolatedKalmanFilter::reprocess_measurement(const MeasData &m) {
  ProcessMeasResult_t res;
  res.rejected = true;

  if (m.obs_type == eObservationType::JOINT_OBSERVATION) {
    res = local_joint_measurement(m);
  } else {
    res = IKalmanFilter::reprocess_measurement(m);
  }

  return res;
}

// KF: Algorithm 8 in [1]
bool IIsolatedKalmanFilter::apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab,
                                              const Timestamp &t_a, const Timestamp &t_b) {
  if (IKalmanFilter::apply_propagation(Phi_II_ab, Q_II_ab, t_a, t_b)) {
    if (add_correction_at_t(t_a, t_b, Phi_II_ab)) {
      check_horizon();
      return true;
    }
    else {
      std::cout << "Could not set the correction factor Phi_II_ab=" << Phi_II_ab << std::endl;
    }
  }
  return false;
}

// EKF: Algorithm 8 in [1]
bool IIsolatedKalmanFilter::apply_propagation(ptr_belief &bel_II_apri, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab,
                                              const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
  if (IKalmanFilter::apply_propagation(bel_II_apri, mean_II_b, Phi_II_ab, Q_II_ab, t_a, t_b)) {
    if (add_correction_at_t(t_a, t_b, Phi_II_ab)) {
      check_horizon();
      return true;
    }
    else {
      std::cout << "Could not set the correction factor Phi_II_ab=" << Phi_II_ab << std::endl;
    }
  }
  return false;
}

// KF:  Algorithm 4 in [1]
bool IIsolatedKalmanFilter::apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                                      const Eigen::VectorXd &z, const Timestamp &t) {
  ptr_belief bel_apri;
  if (get_belief_at_t(t, bel_apri)) {
    Eigen::VectorXd r = z - H_II * bel_apri->mean();
    KalmanFilter::CorrectionCfg_t cfg; // TODO: cfg as parameter?
    KalmanFilter::CorrectionResult_t res;

    res = KalmanFilter::correction_step(H_II, R, r, bel_apri->Sigma(), cfg);

    if (!res.rejected) {
      // correction strategy: IMPORTANT: before setting the belief implace!
      apply_correction_at_t(t, bel_apri->Sigma(), res.Sigma_apos);

      // implace correction: no need to insert the belief into the HistBelief!
      bel_apri->correct(res.delta_mean, res.Sigma_apos);
    }
    return !res.rejected;
  }
  return false;
}

// EKF:  Algorithm 4 in [1]
bool IIsolatedKalmanFilter::apply_private_observation(ptr_belief &bel_II_apri, const Eigen::MatrixXd &H_II,
                                                      const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t){

  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(bel_II_apri->Sigma()), "Apri covariance is not PSD at t=" + t.str());

  KalmanFilter::CorrectionCfg_t cfg; // TODO: cfg as parameter?
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_II, R, r, bel_II_apri->Sigma(), cfg);

  if (!res.rejected) {
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(res.Sigma_apos), "Apos covariance is not PSD at t=" + t.str());

    // correction strategy: IMPORTANT: before setting the belief implace!
    if (apply_correction_at_t(t, res.U)) {

      bel_II_apri->correct(res.delta_mean, res.Sigma_apos);
      return true;
    } else {
      return false;
    }
  }
  return false;;

}


Eigen::MatrixXd IIsolatedKalmanFilter::stack_Sigma(const Eigen::MatrixXd &Sigma_II, const Eigen::MatrixXd &Sigma_JJ, const Eigen::MatrixXd &Sigma_IJ) {

  RTV_EXPECT_TRUE_THROW(Sigma_IJ.size() != 0, "empty Sigma_IJ!");
  RTV_EXPECT_TRUE_THROW((Sigma_II.rows() == Sigma_IJ.rows()) && (Sigma_IJ.cols() == Sigma_JJ.cols()), "dimension missmatch!");

  Eigen::MatrixXd C(Sigma_II.rows()+Sigma_JJ.rows(), Sigma_II.cols() + Sigma_JJ.cols());

  C << Sigma_II, Sigma_IJ,
       Sigma_IJ.transpose(), Sigma_JJ;
  return C;
}

// Algorithm 6 in [1]
void IIsolatedKalmanFilter::split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J, Eigen::MatrixXd& Sigma_II,
                 Eigen::MatrixXd& Sigma_JJ, Eigen::MatrixXd& Sigma_IJ) {
  RTV_EXPECT_TRUE_THROW(dim_I > 0 && dim_J > 0, "Dimension insvalid");

  RTV_EXPECT_TRUE_THROW((Sigma.rows() == (long)(dim_I + dim_J)) && (Sigma.cols() == (long)(dim_I + dim_J)), "dimension missmatch!");
  Sigma_II = Sigma.topLeftCorner(dim_I, dim_I);
  Sigma_JJ = Sigma.bottomRightCorner(dim_J, dim_J);
  Sigma_IJ = Sigma.topRightCorner(dim_I, dim_J);
}


// Algorithm 6 in [1]
Eigen::MatrixXd IIsolatedKalmanFilter::stack_apri_covariance(ptr_belief &bel_I_apri, ptr_belief &bel_J_apri,
                                                             const size_t ID_I, const size_t ID_J, const Timestamp &t) {
  Eigen::MatrixXd Sigma_IJ = get_Sigma_IJ_at_t(ID_I, ID_J, t);

  if (Sigma_IJ.size()) {
    return stack_Sigma(bel_I_apri->Sigma(), bel_J_apri->Sigma(), Sigma_IJ);
  }
  else {
    // Not correlated!
    Sigma_IJ = Sigma_IJ.Zero(bel_I_apri->Sigma().rows(), bel_J_apri->Sigma().cols());
    return stack_Sigma(bel_I_apri->Sigma(), bel_J_apri->Sigma(), Sigma_IJ);
  }
}

//  Eq (1) and Algorithm 6 in [1]
void IIsolatedKalmanFilter::set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J,
                                              const Eigen::MatrixXd &Sigma_IJ, const Timestamp &t) {
  if (ID_I < ID_J) {
    ptr_Handler->get(ID_I)->set_CrossCovFact_at_t(t, ID_J, Sigma_IJ);
    ptr_Handler->get(ID_J)->set_CrossCovFact_at_t(t, ID_I, Eigen::MatrixXd::Identity(Sigma_IJ.cols(), Sigma_IJ.cols()));
  } else {
    Eigen::MatrixXd Sigma_JI = Sigma_IJ.transpose();
    ptr_Handler->get(ID_J)->set_CrossCovFact_at_t(t, ID_I, Sigma_JI);
    ptr_Handler->get(ID_I)->set_CrossCovFact_at_t(t, ID_J, Eigen::MatrixXd::Identity(Sigma_JI.cols(), Sigma_JI.cols()));
  }
}

//  Eq (1) and Algorithm 6 in [1]
Eigen::MatrixXd IIsolatedKalmanFilter::get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Timestamp &t) {
  Eigen::MatrixXd  SigmaFact_IJ = ptr_Handler->get(ID_I)->get_CrossCovFact_at_t(t, ID_J);
  Eigen::MatrixXd  SigmaFact_JI = ptr_Handler->get(ID_J)->get_CrossCovFact_at_t(t, ID_I);
  Eigen::MatrixXd Sigma_IJ;
  if (SigmaFact_IJ.size() && SigmaFact_JI.size()) {
    if (ID_I < ID_J) {
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

// KF: Algorithm 6 in [1]
bool IIsolatedKalmanFilter::apply_joint_observation(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &H_II,
                                                    const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &R,
                                                    const Eigen::VectorXd &z, const Timestamp &t) {
  RTV_EXPECT_TRUE_THROW(ptr_Handler->exists(ID_I) && ptr_Handler->exists(ID_J), "IKF instances do not exists!");
  RTV_EXPECT_TRUE_THROW(ID_I == m_ID, "ID_I missmatch! wrong interim master");

  // get individuals a priori beliefs:
  ptr_belief bel_I_apri, bel_J_apri;
  RTV_EXPECT_TRUE_THROW(get_belief_at_t(t, bel_I_apri), "Could not obtain belief");
  RTV_EXPECT_TRUE_THROW(ptr_Handler->get(ID_J)->get_belief_at_t(t, bel_J_apri), "Could not obtain belief");
  RTV_EXPECT_TRUE_THROW(bel_I_apri->timestamp() == bel_J_apri->timestamp(), "Timestamps of bliefs need to match! Did you forgett to propagate first?");

  // stack the measurement sensitivity matrix:
  Eigen::MatrixXd H_joint = utils::horcat(H_II, H_JJ);

  // stack individual's mean:
  Eigen::VectorXd mean_joint = utils::vertcat_vec(bel_I_apri->mean(), bel_J_apri->mean());

  // residual:
  Eigen::VectorXd r =  z - H_joint * mean_joint;

  return apply_joint_observation(bel_I_apri, bel_J_apri, ID_I, ID_J, H_II, H_JJ, R, r, t);
}

// EKF: Algorithm 6 in [1]
bool IIsolatedKalmanFilter::apply_joint_observation(ptr_belief &bel_I_apri, ptr_belief &bel_J_apri, const size_t ID_I,
                                                    const size_t ID_J, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
                                                    const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t) {
  RTV_EXPECT_TRUE_THROW(ID_I == m_ID, "ID_I missmatch! wrong interim master");


  // stack the measurement sensitivity matrix (again...):
  Eigen::MatrixXd H_joint = utils::horcat(H_II, H_JJ);

  // stack individual's covariances:
  Eigen::MatrixXd Sigma_apri = utils::stabilize_covariance(stack_apri_covariance(bel_I_apri, bel_J_apri, ID_I, ID_J, t));
  RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(Sigma_apri), "Joint apri covariance is not PSD at t=" + t.str());


  KalmanFilter::CorrectionCfg_t cfg;
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_joint, R, r, Sigma_apri, cfg);
  if (!res.rejected) {
    RTV_EXPECT_TRUE_MSG(utils::is_positive_semidefinite(res.Sigma_apos), "Joint apos covariance is not PSD at t=" + t.str());
    size_t dim_I = bel_I_apri->Sigma().rows();
    size_t dim_J = bel_J_apri->Sigma().rows();

    // split covariance
    Eigen::MatrixXd Sigma_II_apos, Sigma_JJ_apos, Sigma_IJ_apos;
    split_Sigma(res.Sigma_apos, dim_I, dim_J, Sigma_II_apos, Sigma_JJ_apos, Sigma_IJ_apos);

    // IMPORTANT: keep order! before setting cross-covariance factors and beliefs implace!
    // 1) add correction terms in the appropriate correction buffers!
    apply_correction_at_t(t, bel_I_apri->Sigma(), Sigma_II_apos);
    ptr_Handler->get(ID_J)->apply_correction_at_t(t, bel_J_apri->Sigma(), Sigma_JJ_apos);

    // 2) set a corrected factorized a posterioiry cross-covariance
    set_Sigma_IJ_at_t(ID_I, ID_J, Sigma_IJ_apos, t);

    // 3) correct beliefs implace!
    bel_I_apri->correct(res.delta_mean.topLeftCorner(dim_I, 1), Sigma_II_apos);
    bel_J_apri->correct(res.delta_mean.bottomRightCorner(dim_J, 1), Sigma_JJ_apos);

  }
  return !res.rejected;
}



} // ns mmsf
