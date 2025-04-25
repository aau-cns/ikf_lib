/******************************************************************************
* FILENAME:     MeasData.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     19.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Measurement/MeasData.hpp>

namespace ikf {

std::string to_string(const eObservationType ot) {
  switch(ot) {
    case eObservationType::UNKNOWN:
      return "UNKNOWN";
    case eObservationType::PROPAGATION:
      return "PROP";
    case eObservationType::PRIVATE_OBSERVATION:
      return "PRIV";
    case eObservationType::JOINT_OBSERVATION:
      return "JOINT";
    default:
      break;

  }
  return "";
}

bool MeasData::has_meas_noise() const {
  if (R.size() && R.norm() > 0.00000000001) {
      return true;
  }
  return false;
}

Eigen::MatrixXd MeasData::get_R() const {
  if(R.cols() == 1) {
    return R.asDiagonal();
  } else {
    return R;
  }

}

std::ostream &operator<<(std::ostream &out, const MeasData &obj) {
  out << "MeasData:";
  out << std::left;
  out << " t_m=" << std::setw(16) << obj.t_m.str();
  out << ", t_p=" << std::setw(16) << obj.t_p.str();
  out << ", ID=" << std::setw(3)<< obj.id_sensor;
  out << ", meas_type=" << std::left << std::setw(20) << obj.meas_type;
  out << ", meta info="<< std::left << std::setw(12) << obj.meta_info;
  out << ", obs. type=" << std::left  << std::setw(2) << to_string(obj.obs_type);
  out << ", z=[" << std::setprecision(4) <<  obj.z.transpose() <<"]";
  out << ", diag(R)=[" << std::setprecision(4);
  if(obj.R.cols() == 1) {
    out << obj.R.transpose();
  } else {
    out << obj.R.diagonal().transpose();
  }
  out << "]" <<std::internal;
  return out;
}

MeasData MeasData::lin_interpolate(const MeasData &m_a, const MeasData &m_c, const Timestamp &t_b) {
  RTV_EXPECT_TRUE_MSG((m_a.id_sensor == m_c.id_sensor), "wrong sensor IDs!");
  RTV_EXPECT_TRUE_MSG((m_a.obs_type == m_c.obs_type), "wrong observation types!");

  double const dt_ac = m_c.t_m.to_sec() - m_a.t_m.to_sec();
  double const dt_ab = t_b.to_sec() - m_a.t_m.to_sec();
  double const ratio = dt_ab /dt_ac;

  MeasData m_b = m_a;
  m_b.t_m = t_b;
  m_b.t_p = t_b;
  m_b.z = m_a.z + (m_c.z - m_a.z)*ratio;
  m_b.R = m_a.R + (m_c.R - m_a.R)*ratio;
  return m_b;
}

std::string MeasData::str() const {
  std::stringstream ss;
  ss << *this;
  return ss.str();
}

std::string ikf::MeasData::str_short() const {
  std::stringstream out;
  out << "MeasData:";
  out << std::left;
  out << " t_m=" << std::setw(16) << t_m.str();
  out << ", ID=" << std::setw(3) << id_sensor;
  out << ", meas_type=" << std::left << std::setw(20) << meas_type;
  return out.str();
}

}  // namespace ikf
