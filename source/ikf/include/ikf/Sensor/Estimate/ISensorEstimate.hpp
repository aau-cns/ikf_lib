/******************************************************************************
* FILENAME:     ISensorEstimate.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     30.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef ISENSORESTIMATE_HPP
#define ISENSORESTIMATE_HPP
#include <memory>
#include "eigen3/Eigen/Eigen"
#include <ikf/ikf_api.h>
namespace ikf {

class IKF_API ISensorEstimate {
public:
  virtual ~ISensorEstimate() {}

  virtual Eigen::VectorXd  to_vector() = 0;
  virtual bool from_vector(Eigen::VectorXd const &vec) = 0;
  virtual std::shared_ptr<ISensorEstimate> clone() = 0; // important in prediction/propagation
  virtual std::shared_ptr<ISensorEstimate> interpolate(std::shared_ptr<ISensorEstimate> obj_a,
                                                       std::shared_ptr<ISensorEstimate> obj_b,
                                                       double const i) = 0; // % returns a new object!
  virtual void correct(Eigen::VectorXd const& dx) = 0; // inplace and accoring to the error definiton!

};

} // namespace ikf

#endif // ISENSORESTIMATE_HPP
