/******************************************************************************
* FILENAME:     IBelief.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     02.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Sensor/Estimate/IBelief.hpp>
namespace ikf {

IBelief::~IBelief() {}

size_t IBelief::es_dim() const { return m_es_dim; }

size_t IBelief::ns_dim() const { return m_ns_dim; }

size_t IBelief::unique_ID() const { return m_unique_ID; }

const Timestamp &IBelief::timestamp() const { return m_timestamp; }

void IBelief::set_timestamp(const Timestamp &t) { m_timestamp = t; }

const BeliefOptions &IBelief::options() const { return m_options; }

void IBelief::set_options(const BeliefOptions &o) { m_options = o; }

std::shared_ptr<IBelief> IBelief::apply_init_strategy(const std::shared_ptr<IBelief> &bel_0, const eInitStrategies type, const int seed) {
  if (seed != 0) {
    // set the seed of the random variable generator to something...
    // TODO: we need a multivariate normal random generator!
  }
  return bel_0;
}


}
