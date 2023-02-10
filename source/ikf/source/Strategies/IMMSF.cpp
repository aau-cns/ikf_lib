/******************************************************************************
* FILENAME:     IMMSF.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Strategies/IMMSF.hpp>
namespace ikf {

IMMSF::IMMSF(std::shared_ptr<IInstanceHandler> ptr_hdl, const std::string &name, size_t ID, const double horizon_sec) : ptr_FC_hdl(ptr_hdl), HistBelief(horizon_sec), m_name(name), m_ID(ID), max_time_horizon_sec(horizon_sec)
{

}

Timestamp IMMSF::current_t() const {
  Timestamp t;
  HistBelief.get_latest_t(t);
  return t;
}

bool IMMSF::exist_belief_at_t(const Timestamp &t) const {
  return HistBelief.exist_at_t(t);
}

ptr_belief IMMSF::get_belief_at_t(const Timestamp &t) const {
  ptr_belief bel;
  if (!HistBelief.get_at_t(t, bel)) {
    std::cout << "IMMSF::get_belief_at_t: could not find belief at t=" << t << std::endl;
  }
  return bel;
}

bool IMMSF::get_belief_at_t(const Timestamp &t, ptr_belief &bel) {
  return HistBelief.get_at_t(t, bel);
}

void IMMSF::set_belief_at_t(const ptr_belief &bel, const Timestamp &t){
  HistBelief.insert(bel, t);
}

bool IMMSF::correct_belief_at_t(const Eigen::VectorXd &mean_corr, const Eigen::MatrixXd &Sigma_apos, const Timestamp &t){
  ptr_belief bel;
  bool res = get_belief_at_t(t, bel);
  if (res) {
    bel->correct(mean_corr, Sigma_apos);
    //set_belief_at_t(bel, t);
  }
  return res;
}

bool IMMSF::get_belief_before_t(const Timestamp &t, ptr_belief &bel, Timestamp &t_before)
{
  TStampedData<ptr_belief> tData;
  bool res = HistBelief.get_before_t(t, tData);
  bel = tData.data;
  t_before = tData.stamp;
  return res;
}

Eigen::VectorXd IMMSF::get_mean_at_t(const Timestamp &t) const {
  ptr_belief bel = get_belief_at_t(t);
  return bel->mean();
}

Eigen::MatrixXd IMMSF::get_Sigma_at_t(const Timestamp &t) const {
  ptr_belief bel = get_belief_at_t(t);
  return bel->Sigma();
}

void IMMSF::reset() {
  HistBelief.clear();
}

void IMMSF::remove_beliefs_after_t(const Timestamp &t) {
  HistBelief.remove_after_t(t);
}

void IMMSF::set_horizon(const double t_hor) {
  max_time_horizon_sec = t_hor;
  HistBelief.set_horizon(t_hor);
}

void IMMSF::check_horizon() {
  HistBelief.check_horizon();
}


} // ns mmsf
