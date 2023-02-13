/******************************************************************************
* FILENAME:     IIKF_DICO.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
*
*  References:
*  [1] Roland Jung and Stephan Weiss, "Scalable Recursive Distributed Collaborative State Estimation for Aided
Inertial Navigation", Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), IEEE, Xi’an, 2021.
*  [2] Lukas Luft et al. "Recursive decentralized localization for multi-robot systems with asynchronous pairwise communication", 2018, IJRR
******************************************************************************/
#ifndef IIKF_DICO_HPP
#define IIKF_DICO_HPP
#include <ikf/ikf_api.h>
#include <eigen3/Eigen/Eigen>
#include <unordered_map>
#include <ikf/Strategies/IMMSF.hpp>
#include <ikf/Container/TTimeHorizonBuffer.hpp>



namespace ikf {

// DICO= Decoupled Input, Coupled Output
class IKF_API IIKF_DICO: public IMMSF {
public:
  IIKF_DICO(std::shared_ptr<IInstanceHandler> ptr_hdl,
             std::string const &name,
             size_t ID = 0,
             double const horizon_sec=1.0);

  /// overriding/extending virtual functions
  void reset();
  virtual void remove_beliefs_after_t(Timestamp const& t);
  virtual void set_horizon(double const t_hor);
  virtual void check_horizon();

  Eigen::MatrixXd get_CrossCovFact_at_t(Timestamp const& t, size_t unique_ID) const;
  Eigen::MatrixXd get_CrossCovFact_before_t(Timestamp const& t, size_t unique_ID) const;
  void set_CrossCovFact_at_t(Timestamp const& t, size_t const unique_ID, Eigen::MatrixXd const& ccf);
  // [1]
  void propagate_CrossCovFact(Timestamp const& t_a, Timestamp const& t_b, Eigen::MatrixXd const& M_a_b);

  // [2]
  static void set_crosscov(std::shared_ptr<IIKF_DICO> const& pIKF_I,
                           std::shared_ptr<IIKF_DICO> const& pIKF_J,
                           size_t const ID_I, size_t const ID_J,
                           Eigen::MatrixXd const& Sigma_IJ, Timestamp const& t);
  // [2]
  static Eigen::MatrixXd get_crosscov(std::shared_ptr<IIKF_DICO> const& pIKF_I,
                                      std::shared_ptr<IIKF_DICO> const& pIKF_J,
                                      size_t const ID_I, size_t const ID_J,
                                      Timestamp const& t);

protected:
  std::unordered_map<size_t, TTimeHorizonBuffer<Eigen::MatrixXd>> HistCrossCovFactors;
}; // class IIKF_DICO

} // namespace ikf

#endif // IIKF_DICO_HPP
