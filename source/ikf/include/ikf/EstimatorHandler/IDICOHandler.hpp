/******************************************************************************
 * FILENAME:     IDICOHandler.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     10.08.2023
 *
 *  Copyright (C) 2023
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#ifndef IKF_IDICOHANDLER_HPP
#define IKF_IDICOHANDLER_HPP
#include <ikf/Estimator/IIsolatedKalmanFilter.hpp>
#include <ikf/ikf_api.h>
#include <memory>
#include <mutex>

namespace ikf {

using namespace std::chrono_literals;

struct IKF_API ApplyObsResult_t {
  eMeasStatus status{eMeasStatus::REJECTED};
  Eigen::VectorXd residual;

  ApplyObsResult_t() = default;
  ApplyObsResult_t(eMeasStatus const& s) : status(s) {}
  ApplyObsResult_t(eMeasStatus const& s, Eigen::VectorXd const& r) : status(s), residual(r) {}
};

///
/// \brief The IDICOHandler class
/// Abstract base class to hold N-IKF instances defines methods which are issued by the IKF instances.
/// IKF instances maintain a shared_ptr to this instance handler to perform  the "DICO" fusion within the
/// IDICOHandler (meaning that the propagation can be performed isolated/decoupled from the rest)
///
class IKF_API IDICOHandler {
public:
  IDICOHandler(double const horizon_sec = 1.0);
  ~IDICOHandler() {}

  bool add(pIKF_t p_IKF);
  virtual pIKF_t get(const size_t ID);
  bool remove(const size_t ID);
  bool exists(const size_t ID);
  std::vector<size_t> get_instance_ids();
  std::map<size_t, std::string> get_instance_types();

  double horizon_sec() const;
  void set_horizon(double const t_hor);
  bool reset();
  bool reset_buffer();

  // TODO: hack needed for mmsf::SensorEstimatorHandler
  virtual size_t get_propagation_sensor_ID(size_t const ID = 0);
  void set_propagation_sensor_ID(size_t const ID);
  virtual std::string get_type_by_ID(size_t const ID = 0);

  // THREAD SAFE INTERFACE
  virtual bool set_belief_at_t(size_t const ID, pBelief_t& bel, Timestamp const& t);
  virtual bool get_belief_at_t(size_t const ID, Timestamp const& t, pBelief_t& bel,
                               eGetBeliefStrategy const type = eGetBeliefStrategy::EXACT);

  virtual bool get_beliefs_at_t(std::vector<size_t> const& IDs, std::vector<eGetBeliefStrategy> const& types,
                                Timestamp const& t, std::map<size_t, pBelief_t>& beliefs);
  virtual Eigen::MatrixXd get_CrossCovFact_at_t(size_t const ID_I, Timestamp const& t, size_t const ID_J);
  bool set_CrossCovFact_at_t(size_t const ID_I, Timestamp const& t, size_t const ID_J, Eigen::MatrixXd const& FFC_IJ);
  virtual bool apply_correction_at_t(size_t const ID, Timestamp const& t, Eigen::MatrixXd const& Factor);

  virtual ProcessMeasResult_vec_t process_measurement(MeasData const& m);

  /// Generic fusion algorithm for M-participants:
  /// - the state dim can be infered from the cols of the H matrices
  /// - IDs of particants can be obtained through the dictionary keys.
  virtual bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::MatrixXd& R,
                                 const Eigen::VectorXd& r, const Timestamp& t, const KalmanFilter::CorrectionCfg_t& cfg)
    = 0;
  virtual bool apply_observation(std::map<size_t, Eigen::MatrixXd> const& dict_H, const Eigen::VectorXd& z,
                                 const Eigen::MatrixXd& R, const Timestamp& t, const KalmanFilter::CorrectionCfg_t& cfg)
    = 0;
  virtual ApplyObsResult_t apply_observation(const Eigen::MatrixXd& R, const Eigen::VectorXd& z, const Timestamp& t,
                                             IIsolatedKalmanFilter::h_joint const& h, std::vector<size_t> const& IDs,
                                             const KalmanFilter::CorrectionCfg_t& cfg)
    = 0;

  virtual bool insert_measurement(MeasData const& m, Timestamp const& t);

  void print_HistMeas(std::ostream& out, size_t max);

  virtual ProcessMeasResult_vec_t redo_updates_after_t(const Timestamp& t);
  virtual bool schedule_redo_update(const Timestamp& t, bool const after = true);

protected:
  void sort_measurements_from_t(Timestamp const& t);
  /////////////////////////////////////////////////////
  /// Interface for IKF handles to reprocess measurements
  // TMultiHistoryBuffer<MeasData> get_measurements_from_t(Timestamp const& t);
  // TMultiHistoryBuffer<MeasData> get_measurements_after_t(Timestamp const& t);
  virtual bool is_order_violated(MeasData const& m);
  virtual bool discard_measurement(MeasData const& m);
  virtual ProcessMeasResult_vec_t redo_updates_from_t(const Timestamp& t);
  virtual ProcessMeasResult_t delegate_measurement(MeasData const& m);
  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void remove_beliefs_from_t(Timestamp const& t);

  std::unordered_map<size_t, std::shared_ptr<IIsolatedKalmanFilter>> id_dict;
  bool m_handle_delayed_meas = true;
  TTimeHorizonBuffer<MeasData, TMultiHistoryBuffer<MeasData>> HistMeas;     // measurements that have ben processed
  TTimeHorizonBuffer<bool> Hist_OOO_Flag;  // flags measurements timestamps that have not been processed!
  double m_horzion_sec;

  size_t m_PropSensor_ID{0};
  TTimeHorizonBuffer<bool> HistRedoUpdateRequest;

  std::chrono::milliseconds mtx_timeout_ms{25ms};
  std::recursive_timed_mutex m_mtx;
  std::mutex m_mtx_histRUR;
};

typedef std::shared_ptr<IDICOHandler> pDICOHandler_t;
}  // namespace ikf

#endif  // IKF_IDICOHANDLER_HPP
