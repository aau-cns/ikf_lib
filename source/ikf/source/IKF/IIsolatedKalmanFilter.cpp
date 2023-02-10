/******************************************************************************
* FILENAME:     IsolatedKalmanFilter.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     03.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/IKF/IIsolatedKalmanFilter.hpp>
#include <ikf/IKF/IsolatedKalmanFilterHandler.hpp>
#include <ikf/Sensor/Estimator/KalmanFilter.hpp>
#include <ikf/utils/eigen_utils.hpp>


namespace ikf {

IIsolatedKalmanFilter::IIsolatedKalmanFilter(std::shared_ptr<IsolatedKalmanFilterHandler> ptr_Handler, const std::string &name, const size_t ID, const bool handle_delayed_meas, const double horizon_sec) : IFilterInstance(horizon_sec, handle_delayed_meas), ptr_Handler(ptr_Handler), HistCorr(horizon_sec), m_name(name), m_ID(ID) {

}

ProcessMeasResult_t IIsolatedKalmanFilter::process_measurement(const MeasData &m) {
  // propagation and private -> to filter instance
  // joint -> use ptr_Handler and two filter instances: The CIH must provide others belief and ccf

  ProcessMeasResult_t res;
  res.rejected = true;


  if (m.obs_type == eObservationType::LOCAL_JOINT) {
    res = local_joint_measurement(m);
    if (m_handle_delayed_meas) {
      if (!res.rejected && HistMeas.exist_after_t(m.t_m)) {

        // TODO: notify other instances to redo their updates!
        ///ptr_Handler->redo_updates_after_t(res.ID_participants, m.t);
        redo_updates_after_t(m.t_m);
      }
      HistMeas.insert(m, m.t_m);
    }
  } else {
    res = IFilterInstance::process_measurement(m);
  }

  return res;
}

void IIsolatedKalmanFilter::initialize(ptr_belief bel_init, const Timestamp &t) {
  reset();
  HistBelief.insert(bel_init, t);
}

bool IIsolatedKalmanFilter::get_apri_belief_and_fcc_at_t(const Timestamp &t, size_t ID_other, ptr_belief ptr_bel, Eigen::MatrixXd &factorized_cross_cov) {
  return IFilterInstance::get_belief_at_t(t, ptr_bel) && get_CrossCovFact_at_t(t, ID_other, factorized_cross_cov);
}

bool IIsolatedKalmanFilter::set_apos_belief_and_fcc_at_t(const Timestamp &t, size_t ID_other, ptr_belief ptr_bel, Eigen::MatrixXd &factorized_cross_cov)
{
  IFilterInstance::set_belief_at_t(ptr_bel, t);
  set_CrossCovFact_at_t(t, ID_other, factorized_cross_cov);
  return true;
  // TODO: technically we could start redoing updates here!
}

bool IIsolatedKalmanFilter::set_apos_belief_and_fcc_at_t(const Timestamp &t, size_t ID_other, const Eigen::VectorXd &mean_corr, const Eigen::MatrixXd &Sigma, const Eigen::MatrixXd &factorized_cross_cov)
{
  bool res = IFilterInstance::correct_belief_at_t(mean_corr, Sigma, t);
  if (res) {
    set_CrossCovFact_at_t(t, ID_other, factorized_cross_cov);
  }
  return res;
}

bool IIsolatedKalmanFilter::redo_updates_after_t(const Timestamp &t) {
  remove_after_t(t);
  Timestamp t_after, t_last;
  if (HistMeas.get_after_t(t, t_after) &&  HistMeas.get_latest_t(t_last)) {
    HistMeas.foreach_between_t1_t2(t_after, t_last, [this](MeasData const&m){ this->process_measurement(m); });
    return true;
  }
  return false;
}

bool IIsolatedKalmanFilter::clone_fccs(const size_t ID_old, const size_t ID_new) {
  auto iter = HistCrossCovFactors.find(ID_old);
  if (iter != HistCrossCovFactors.end()) {
    HistCrossCovFactors.emplace(ID_new, iter->second.clone());
    return true;
  }
  return false;
}

std::vector<size_t> IIsolatedKalmanFilter::get_correlated_IDs() const {
  std::vector<size_t> IDs;
  IDs.reserve(HistCrossCovFactors.size());
  for (auto const& elem : HistCrossCovFactors) {
    IDs.push_back(elem.first);
  }
  return IDs;
}

bool IIsolatedKalmanFilter::get_CrossCovFact_at_t(const Timestamp &t, size_t unique_ID, Eigen::MatrixXd &FFC){
  FFC = get_CrossCovFact_at_t(t, unique_ID);
  return FFC.size() > 0;
}

Eigen::MatrixXd IIsolatedKalmanFilter::get_CrossCovFact_at_t(const Timestamp &t, size_t unique_ID) {
  Eigen::MatrixXd mat;
  auto iter = HistCrossCovFactors.find(unique_ID);
  if (iter != HistCrossCovFactors.end()) {
    bool res = iter->second.get_at_t(t, mat);
    if (!res) {

      // forward propagate FCC from previous to current timestamp and insert it in the buffer!
      Timestamp t_prev;
      res = iter->second.get_before_t(t, t_prev);
      if (res) {
        res = iter->second.get_at_t(t_prev, mat);
        Eigen::MatrixXd M_a_b = compute_correction(t_prev, t);
        if (res && M_a_b.size() > 0 ) {
          mat = M_a_b * mat;
          set_CrossCovFact_at_t(t, unique_ID, mat);
        }
        else {
          std::cout << "IMMSF.get_CrossCovFact_at_t(): could not compute correction between t_prev=" << t_prev << " and t_curr=" << t << std::endl;
        }
      }
      else {
        std::cout << "IMMSF.get_CrossCovFact_at_t(): could not find elem for id=" << unique_ID << " at t=" << t << std::endl;
        std::cout << "IMMSF.get_CrossCovFact_at_t(): could not find elem for id=" << unique_ID << " at t_prev=" << t_prev << std::endl;
      }
    }
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

void IIsolatedKalmanFilter::reset() {
  IFilterInstance::reset();
  HistCorr.clear();
  HistCrossCovFactors.clear();
}

void IIsolatedKalmanFilter::remove_after_t(const Timestamp &t) {
  IFilterInstance::remove_beliefs_after_t(t);
  HistCorr.remove_after_t(t);
  for (auto& elem : HistCrossCovFactors){
    elem.second.remove_after_t(t);
  }
}

void IIsolatedKalmanFilter::set_horizon(const double t_hor) {
  IFilterInstance::set_horizon(t_hor*2);
  HistCorr.set_horizon(t_hor*2);

  // measurement horizon can be max half the horizon of beliefs!
  HistMeas.set_horizon(t_hor);
  for (auto& elem : HistCrossCovFactors){
    elem.second.set_horizon(t_hor*2);
  }
}

void IIsolatedKalmanFilter::check_horizon() {
  IFilterInstance::check_horizon();
  HistCorr.check_horizon();
  for (auto& elem : HistCrossCovFactors){
    elem.second.check_horizon();
  }
}

Eigen::MatrixXd IIsolatedKalmanFilter::compute_correction(const Timestamp &t_a, const Timestamp &t_b) const {
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
                                                              [](Eigen::MatrixXd const&A, Eigen::MatrixXd const&B){
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

bool IIsolatedKalmanFilter::add_correction_at_t(const Timestamp &t_b, const Eigen::MatrixXd &Phi_a_b) {
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

bool IIsolatedKalmanFilter::apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Factor){
  RTV_EXPECT_TRUE_MSG(Factor.cols() == Factor.rows(), "Factor must be a square matrix!");
  Eigen::MatrixXd mat;
  if (HistCorr.get_at_t(t, mat)) {
    mat = Factor * mat;
    HistCorr.insert(mat, t);
    return true;
  }
  else {
    HistCorr.insert(Factor, t);
    std::cout << "IMMF::apply_correction_at_t(): no element found for timestamps:[" << t << "]" << std::endl;
    return false;
  }
}

bool IIsolatedKalmanFilter::apply_correction_at_t(const Timestamp &t, const Eigen::MatrixXd &Sigma_apri, const Eigen::MatrixXd Sigma_apos){
  Eigen::MatrixXd Lambda = Sigma_apos * Sigma_apri.inverse();
  return apply_correction_at_t(t, Lambda);
}

bool IIsolatedKalmanFilter::apply_propagation(const Eigen::MatrixXd &Phi_II_ab, const Eigen::MatrixXd &Q_II_ab,
                                              const Timestamp &t_a, const Timestamp &t_b) {
  if (IFilterInstance::apply_propagation(Phi_II_ab, Q_II_ab, t_a, t_b)) {
    if (add_correction_at_t(t_b, Phi_II_ab)) {
      return true;
    }
    else {
      std::cout << "Could not set the correction factor Phi_II_ab=" << Phi_II_ab << std::endl;
    }
  }
  return false;
}

bool IIsolatedKalmanFilter::apply_propagation(ptr_belief &bel_II_apri, const Eigen::VectorXd &mean_II_b, const Eigen::MatrixXd &Phi_II_ab,
                                              const Eigen::MatrixXd &Q_II_ab, const Timestamp &t_a, const Timestamp &t_b) {
  if (IFilterInstance::apply_propagation(bel_II_apri, mean_II_b, Phi_II_ab, Q_II_ab, t_a, t_b)) {
    if (add_correction_at_t(t_b, Phi_II_ab)) {
      return true;
    }
    else {
      std::cout << "Could not set the correction factor Phi_II_ab=" << Phi_II_ab << std::endl;
    }
  }
  return false;
}

bool IIsolatedKalmanFilter::apply_private_observation(const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &R,
                                                      const Eigen::VectorXd &z, const Timestamp &t) {
  ptr_belief bel_apri;
  if (get_belief_at_t(t, bel_apri)) {
    KalmanFilter::CorrectionCfg_t cfg; // TODO: cfg as parameter?
    KalmanFilter::CorrectionResult_t res;
    Eigen::VectorXd r = z - H_II * bel_apri->mean();
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

bool IIsolatedKalmanFilter::apply_private_observation(ptr_belief &bel_II_apri, const Eigen::MatrixXd &H_II,
                                                      const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t){

  KalmanFilter::CorrectionCfg_t cfg; // TODO: cfg as parameter?
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_II, R, r, bel_II_apri->Sigma(), cfg);

  if (!res.rejected) {
    // correction strategy: IMPORTANT: before setting the belief implace!
    if (apply_correction_at_t(t, bel_II_apri->Sigma(), res.Sigma_apos)) {

      bel_II_apri->correct(res.delta_mean, res.Sigma_apos);
      return true;
    } else {
      return false;
    }
  }
  return false;;

}


Eigen::MatrixXd stack_Sigma(const Eigen::MatrixXd &Sigma_II, const Eigen::MatrixXd &Sigma_JJ, const Eigen::MatrixXd &Sigma_IJ) {

  RTV_EXPECT_TRUE_THROW(Sigma_IJ.size() != 0, "empty Sigma_IJ!");
//  if (Sigma_IJ.size() == 0) {
//    Sigma_IJ = Sigma_IJ.Zero(Sigma_II.rows(), Sigma_JJ.cols());
//  }

  RTV_EXPECT_TRUE_THROW((Sigma_II.rows() == Sigma_IJ.rows()) && (Sigma_IJ.cols() == Sigma_JJ.cols()), "dimension missmatch!");

  Eigen::MatrixXd C(Sigma_II.rows()+Sigma_JJ.rows(), Sigma_II.cols() + Sigma_JJ.cols());

  C << Sigma_II, Sigma_IJ,
       Sigma_IJ.transpose(), Sigma_JJ;
  return C;
}

void split_Sigma(Eigen::MatrixXd const& Sigma, size_t const dim_I, size_t const dim_J, Eigen::MatrixXd& Sigma_II,
                 Eigen::MatrixXd& Sigma_JJ, Eigen::MatrixXd& Sigma_IJ) {
  RTV_EXPECT_TRUE_THROW(dim_I > 0 && dim_J > 0, "Dimension insvalid");

  RTV_EXPECT_TRUE_THROW((Sigma.rows() == (long)(dim_I + dim_J)) && (Sigma.cols() == (long)(dim_I + dim_J)), "dimension missmatch!");
  Sigma_II = Sigma.topLeftCorner(dim_I, dim_I);
  Sigma_JJ = Sigma.bottomRightCorner(dim_J, dim_J);
  Sigma_IJ = Sigma.topRightCorner(dim_I, dim_J);
}



Eigen::MatrixXd IIsolatedKalmanFilter::stack_apri_covariance(ptr_belief &bel_I_apri, ptr_belief &bel_J_apri,
                                                             const size_t ID_I, const size_t ID_J, const Timestamp &t) {
  Eigen::MatrixXd Sigma_IJ = get_Sigma_IJ_at_t(ID_I, ID_J, t);

  if (Sigma_IJ.size()) {
    return stack_Sigma(bel_I_apri->Sigma(), bel_J_apri->Sigma(), Sigma_IJ);
  }
  else {
    // Not correlated!
    Sigma_IJ = Sigma_IJ.Zero(bel_I_apri->Sigma().rows(), bel_J_apri->Sigma().cols());
    // (Sigma_II.rows() == Sigma_IJ.rows()) && (Sigma_IJ.cols() == Sigma_JJ.cols())
    return stack_Sigma(bel_I_apri->Sigma(), bel_J_apri->Sigma(), Sigma_IJ);
  }
}


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

bool IIsolatedKalmanFilter::apply_joint_observation(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd &H_II,
                                                    const Eigen::MatrixXd &H_JJ, const Eigen::MatrixXd &R,
                                                    const Eigen::VectorXd &z, const Timestamp &t) {
  RTV_EXPECT_TRUE_THROW(ptr_Handler->exists(ID_I) && ptr_Handler->exists(ID_J), "IKF instances do not exists!");
  RTV_EXPECT_TRUE_THROW(ID_I == m_ID, "ID_I missmatch! wrong interim master");

  // get individuals a priori beliefs:
  ptr_belief bel_I_apri, bel_J_apri;
  RTV_EXPECT_TRUE_THROW(ptr_Handler->get(ID_J)->get_belief_at_t(t, bel_J_apri), "Could not obtain belief");
  RTV_EXPECT_TRUE_THROW(get_belief_at_t(t, bel_I_apri), "Could not obtain belief");

  // stack the measurement sensitivity matrix:
  Eigen::MatrixXd H_joint = utils::horcat(H_II, H_JJ);
  // stack individual's covariances:
  Eigen::MatrixXd Sigma_joint = utils::stabilize_covariance(stack_apri_covariance(bel_I_apri, bel_J_apri, ID_I, ID_J, t));
  // stack individual's mean:
  Eigen::VectorXd mean_joint = utils::vertcat_vec(bel_I_apri->mean(), bel_J_apri->mean());

  // residual:
  Eigen::VectorXd r =  z - H_joint * mean_joint;

  KalmanFilter::CorrectionCfg_t cfg;
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_joint, R, r, Sigma_joint, cfg);
  if (!res.rejected) {

    RTV_EXPECT_TRUE_THROW(utils::is_positive_semidefinite(res.Sigma_apos), "Joint apos covariance is not PSD!");

    // split covariance
    Eigen::MatrixXd Sigma_II_apos, Sigma_JJ_apos, Sigma_IJ_apos;
    split_Sigma(res.Sigma_apos, bel_I_apri->es_dim(), bel_J_apri->es_dim(), Sigma_II_apos, Sigma_JJ_apos, Sigma_IJ_apos);

    // IMPORTANT: before setting the belief implace!
    // add correction terms in the appropriate correction buffers!
    apply_correction_at_t(t, bel_I_apri->Sigma(), Sigma_II_apos);
    ptr_Handler->get(ID_J)->apply_correction_at_t(t, bel_J_apri->Sigma(), Sigma_JJ_apos);

    // set a corrected factorized a posterioiry cross-covariance
    set_Sigma_IJ_at_t(ID_I, ID_J, Sigma_IJ_apos, t);

    // correct beliefs implace!
    bel_I_apri->correct(res.delta_mean.topLeftCorner(bel_I_apri->es_dim(), 1), Sigma_II_apos);
    bel_J_apri->correct(res.delta_mean.bottomRightCorner(bel_J_apri->es_dim(), 1), Sigma_JJ_apos);

  }
  return !res.rejected;
}

bool IIsolatedKalmanFilter::apply_joint_observation(ptr_belief &bel_II_apri, ptr_belief &bel_JJ_apri, const size_t ID_I,
                                                    const size_t ID_J, const Eigen::MatrixXd &H_II, const Eigen::MatrixXd &H_JJ,
                                                    const Eigen::MatrixXd &R, const Eigen::VectorXd &r, const Timestamp &t) {
  RTV_EXPECT_TRUE_THROW(ptr_Handler->exists(ID_I) && ptr_Handler->exists(ID_J), "IKF instances do not exists!");
  RTV_EXPECT_TRUE_THROW(ID_I == m_ID, "ID_I missmatch! wrong interim master");


  // stack the measurement sensitivity matrix:
  Eigen::MatrixXd H_joint = utils::horcat(H_II, H_JJ);
  // stack individual's covariances:
  Eigen::MatrixXd Sigma_joint = utils::stabilize_covariance(stack_apri_covariance(bel_II_apri, bel_JJ_apri, ID_I, ID_J, t));

  KalmanFilter::CorrectionCfg_t cfg;
  KalmanFilter::CorrectionResult_t res;
  res = KalmanFilter::correction_step(H_joint, R, r, Sigma_joint, cfg);
  if (!res.rejected) {

    RTV_EXPECT_TRUE_THROW(utils::is_positive_semidefinite(res.Sigma_apos), "Joint apos covariance is not PSD!");

    // split covariance
    Eigen::MatrixXd Sigma_II_apos, Sigma_JJ_apos, Sigma_IJ_apos;
    split_Sigma(res.Sigma_apos, bel_II_apri->es_dim(), bel_JJ_apri->es_dim(), Sigma_II_apos, Sigma_JJ_apos, Sigma_IJ_apos);

    // IMPORTANT: before setting the belief implace!
    // add correction terms in the appropriate correction buffers!
    apply_correction_at_t(t, bel_II_apri->Sigma(), Sigma_II_apos);
    ptr_Handler->get(ID_J)->apply_correction_at_t(t, bel_JJ_apri->Sigma(), Sigma_JJ_apos);

    // set a corrected factorized a posterioiry cross-covariance
    set_Sigma_IJ_at_t(ID_I, ID_J, Sigma_IJ_apos, t);

    // correct beliefs implace!
    bel_II_apri->correct(res.delta_mean.topLeftCorner(bel_II_apri->es_dim(), 1), Sigma_II_apos);
    bel_JJ_apri->correct(res.delta_mean.bottomRightCorner(bel_JJ_apri->es_dim(), 1), Sigma_JJ_apos);

  }
  return !res.rejected;
}



} // ns mmsf
