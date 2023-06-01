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
  IsolatedKalmanFilterHandler(bool const handle_delayed=true, double const horizon_sec=1.0);
  ~IsolatedKalmanFilterHandler() = default;

  bool add(ptr_IKF p_IKF);
  ptr_IKF get(const size_t ID);
  bool remove(const size_t ID);
  bool exists(const size_t ID);
  std::vector<size_t> get_instance_ids();
  bool handle_delayed_meas() const;

  virtual bool insert_measurement(MeasData const& m, Timestamp const& t);

  void sort_measurements_from_t(Timestamp const& t);


  ///
  /// \brief process_measurement: If the m_handle_delayed_meas == true, the IKF-Handler is a centralized entity hanndle all incomming measurements and is responsible to handle delayed measurements. If
  /// \param m
  /// \return
  ///
  virtual ProcessMeasResult_t process_measurement(MeasData const& m);

  /////////////////////////////////////////////////////
  /// Interface for IKF handles to reprocess measurements
  TMultiHistoryBuffer<MeasData> get_measurements_from_t(Timestamp const&t);
  TMultiHistoryBuffer<MeasData> get_measurements_after_t(Timestamp const&t);
  virtual ProcessMeasResult_t reprocess_measurement(MeasData const& m);
  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void remove_beliefs_from_t(Timestamp const& t);
  void reset();

protected:
  bool is_order_violated(MeasData const& m);
  virtual bool redo_updates_from_t(const Timestamp &t);
  virtual bool redo_updates_after_t(const Timestamp &t);


  std::unordered_map<size_t, std::shared_ptr<IIsolatedKalmanFilter>> id_dict;
  bool m_handle_delayed_meas = true;
  TTimeHorizonBuffer<MeasData, TMultiHistoryBuffer<MeasData>> HistMeas;
  double m_horzion_sec;
};


} // ns mmsf

#endif // ISOLATEDKALMANFILTERHANDLER_HPP
