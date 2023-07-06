/******************************************************************************
* FILENAME:     rand_op.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     06.07.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_RAND_OP_HPP
#define IKF_RAND_OP_HPP
#include <ikf/ikf_api.h>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <ikf/utils/GaussianNoiseGen.hpp>

namespace ikf {

template<typename Scalar>
struct IKF_API rand_op {
  ikf::GaussianNoiseGen& m_gen;
  explicit rand_op(ikf::GaussianNoiseGen& gen) : m_gen(gen) {}
  template<typename Index>
  inline const Scalar operator()(Index, Index = 0) const
  {
    return static_cast<Scalar>(m_gen.randn());
  }
};

} // ns ikf
#endif // RAND_OP_HPP
