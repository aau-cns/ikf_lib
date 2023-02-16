/******************************************************************************
* FILENAME:     IKFHandlerStd.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
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
#ifndef IKFHANDLERSTD_HPP
#define IKFHANDLERSTD_HPP
#include <ikf/ikf_api.h>
#include <memory>
#include <ikf/EstimatorStd/IsolatedKalmanFilterStd.hpp>

namespace ikf {

class IKF_API IKFHandlerStd {
    typedef std::shared_ptr<IsolatedKalmanFilterStd> ptr_IKF;
public:
    bool add(ptr_IKF p_IKF);
    bool remove(const size_t ID);
    bool exists(const size_t ID);
    ptr_IKF get(const size_t ID);
  protected:
    std::unordered_map<size_t, ptr_IKF> id_dict;
};

} // ns ikf

#endif // IKFHANDLERSTD_HPP
