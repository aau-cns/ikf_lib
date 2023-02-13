/******************************************************************************
* FILENAME:     IsolatedKalmanFilterHandler.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     03.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef ISOLATEDKALMANFILTERHANDLER_HPP
#define ISOLATEDKALMANFILTERHANDLER_HPP
#include <ikf/ikf_api.h>
#include <memory>
#include <ikf/Estimator/IIsolatedKalmanFilter.hpp>

namespace ikf {


class IKF_API IsolatedKalmanFilterHandler {

  typedef std::shared_ptr<IIsolatedKalmanFilter> ptr_IKF;
public:
  IsolatedKalmanFilterHandler(double const horizon_sec=1.0) : HistMeas(horizon_sec), m_horzion_sec(horizon_sec) {}
  ~IsolatedKalmanFilterHandler() = default;
  ProcessMeasResult_t process_measurement(const MeasData &m);

  void redo_updates_after_t(Timestamp const& t);


  void remove_after_t(Timestamp const& t);

  void disp_measurement_hist();

  bool add(ptr_IKF p_IKF);

  bool remove(const size_t ID);
  bool exists(const size_t ID);

  std::vector<size_t> get_instance_ids();

  std::shared_ptr<IIsolatedKalmanFilter> get(const size_t ID);

  void reset();

protected:
  std::unordered_map<size_t, std::shared_ptr<IIsolatedKalmanFilter>> id_dict;
  bool m_handle_delayed_meas = true;
  TTimeHorizonBuffer<MeasData> HistMeas;
  double m_horzion_sec;
};


} // ns mmsf

#endif // ISOLATEDKALMANFILTERHANDLER_HPP
