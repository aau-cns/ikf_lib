/******************************************************************************
* FILENAME:     IsolatedKalmanFilterHandler.hpp
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
#ifndef ISOLATEDKALMANFILTERHANDLER_HPP
#define ISOLATEDKALMANFILTERHANDLER_HPP
#include <ikf/ikf_api.h>
#include <memory>
#include <ikf/Estimator/IIsolatedKalmanFilter.hpp>

namespace ikf {


class IKF_API IsolatedKalmanFilterHandler {


public:
  IsolatedKalmanFilterHandler(bool const handle_delayed = true, double const horizon_sec = 1.0);
  ~IsolatedKalmanFilterHandler() = default;

  bool add(pIKF_t p_IKF);
  virtual pIKF_t get(const size_t ID);
  bool remove(const size_t ID);
  bool exists(const size_t ID);
  std::vector<size_t> get_instance_ids();
  double horizon_sec() const;
  void set_horizon(double const t_hor);
  void reset();

  bool get_belief_at_t(size_t const ID, Timestamp const& t, pBelief_t& bel,
                       eGetBeliefStrategy const type = eGetBeliefStrategy::EXACT);

  bool get_prop_meas_at_t(size_t const ID, Timestamp const& t, MeasData& m);

  /// \brief process_measurement: If the m_handle_delayed_meas == true, the IKF-Handler is a centralized entity hanndle
  /// all incomming measurements and is responsible to handle delayed measurements. \param m \return
  virtual ProcessMeasResult_t process_measurement(MeasData const& m);

  /// Generic fusion algorithm for M-participants:
  /// - the state dim can be infered from the cols of the H matrices
  /// - IDs of particants can be obtained through the dictionary keys.
  ///
  ///
  virtual bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::MatrixXd& R,
                                 const Eigen::VectorXd& r, const Timestamp& t,
                                 const KalmanFilter::CorrectionCfg_t& cfg);
  virtual bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::VectorXd& z,
                                 const Eigen::MatrixXd& R, const Timestamp& t,
                                 const KalmanFilter::CorrectionCfg_t& cfg);

protected:
  virtual bool insert_measurement(MeasData const& m, Timestamp const& t);

  void sort_measurements_from_t(Timestamp const& t);
  /////////////////////////////////////////////////////
  /// Interface for IKF handles to reprocess measurements
  TMultiHistoryBuffer<MeasData> get_measurements_from_t(Timestamp const& t);
  TMultiHistoryBuffer<MeasData> get_measurements_after_t(Timestamp const& t);
  bool is_order_violated(MeasData const& m);
  virtual bool redo_updates_from_t(const Timestamp &t);
  virtual bool redo_updates_after_t(const Timestamp &t);
  virtual ProcessMeasResult_t delegate_measurement(MeasData const& m);
  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void remove_beliefs_from_t(Timestamp const& t);

  virtual Eigen::MatrixXd get_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, Timestamp const& t);
  virtual void set_Sigma_IJ_at_t(const size_t ID_I, const size_t ID_J, const Eigen::MatrixXd& Sigma_IJ,
                                 const Timestamp& t);

  virtual Eigen::MatrixXd stack_H(const std::map<size_t, Eigen::MatrixXd>& dict_H);

  virtual std::map<size_t, pBelief_t> get_dict_bel(const std::map<size_t, Eigen::MatrixXd>& dict_H, Timestamp const& t);

  Eigen::VectorXd stack_mean(const std::map<size_t, pBelief_t>& dict_bel);
  virtual Eigen::MatrixXd stack_Sigma(const std::map<size_t, pBelief_t>& dict_bel, Timestamp const& t);

  virtual void apply_corrections_at_t(Eigen::MatrixXd& Sigma_apos, const std::map<size_t, pBelief_t>& dict_bel,
                                      Timestamp const& t);

  virtual void split_right_upper_covariance(Eigen::MatrixXd& Sigma, const std::map<size_t, pBelief_t>& dict_bel,
                                            Timestamp const& t);

  void correct_beliefs_implace(Eigen::MatrixXd& Sigma_apos, Eigen::VectorXd& delta_mean,
                               const std::map<size_t, pBelief_t>& dict_bel);

  std::unordered_map<size_t, std::shared_ptr<IIsolatedKalmanFilter>> id_dict;
  bool m_handle_delayed_meas = true;
  TTimeHorizonBuffer<MeasData, TMultiHistoryBuffer<MeasData>> HistMeas;
  double m_horzion_sec;
};  // class IsolatedKalmanFilterHandler

}  // namespace ikf

#endif // ISOLATEDKALMANFILTERHANDLER_HPP
