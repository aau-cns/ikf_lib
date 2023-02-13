/******************************************************************************
* FILENAME:     ICollaborativeInstanceHandler.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     30.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef ICOLLABORATIVEINSTANCEHANDLER_HPP
#define ICOLLABORATIVEINSTANCEHANDLER_HPP
#include <ikf/ikf_api.h>
#include <ikf/Container/Timestamp.hpp>
#include <ikf/Measurement/MeasData.hpp>
#include <ikf/Estimate/IBelief.hpp>
#include <ikf/Strategies/IInstanceHandler.hpp>

namespace ikf
{

class IKF_API ICollaborativeInstanceHandler : public IInstanceHandler {
public:
  virtual ~ICollaborativeInstanceHandler() {}


  virtual bool get_apri_belief_and_fcc_at_t(size_t const ID_i, Timestamp const& t, size_t const ID_j,  ptr_belief  ptr_bel_j, Eigen::MatrixXd & factorized_cross_cov_ji) const = 0;
  virtual bool set_apos_belief_and_fcc_at_t(size_t const ID_i, Timestamp const& t, size_t ID_j, ptr_belief prt_bel_j, Eigen::MatrixXd & factorized_cross_cov_ji) = 0;
  virtual bool set_apos_belief_and_fcc_at_t(size_t const ID_i,  Timestamp const& t, size_t ID_j, Eigen::VectorXd const& mean_corr_j, Eigen::MatrixXd const& Sigma_apos_jj,  Eigen::MatrixXd const& factorized_cross_cov_apos_ji) = 0;
  virtual bool redo_updates_after_t(std::vector<size_t> const& ID_j, Timestamp const& t) = 0;

  virtual std::vector<size_t> get_correlated_IDs(size_t const ID_i) const = 0;

  // this method replicates an exisitng IKF instance
  virtual bool clone(size_t const ID_old);
}; // ICollaborativeInstanceHandler

} // namespace ikf
#endif // ICOLLABORATIVEINSTANCEHANDLER_HPP
