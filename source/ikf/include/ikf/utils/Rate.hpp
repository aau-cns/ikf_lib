/******************************************************************************
* FILENAME:     Rate.hpp
* PURPOSE:      Rate
* AUTHOR:       jungr - Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     14.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef UTILS_RATE_HPP
#define UTILS_RATE_HPP
#include <ikf/ikf_api.h>
#include <thread>
#include <iostream>
#include <chrono>
#include <atomic>
#include <cstdint>

namespace ikf
{
namespace utils
{
  class IKF_API Rate
  {
    public:
    /**
     * create object with desired frequency
     * @param hz desired frequency; 0 for maximum
     */
    Rate(double const& hz);

    /**
     * sleep to achieve desired frequency
     */
    void sleep();

    std::int32_t getProcessTimeMS() const;

    void setRate(double const hz);


    private:

    typedef std::chrono::high_resolution_clock Clock;

    Rate(const Rate&) = delete;

    Rate& operator=(const Rate&) = delete;

    std::atomic<std::uint32_t> mSleep_ms;
    Clock::time_point          mtStart;
    Clock::time_point          mtStop;
    std::chrono::milliseconds  mProcessTime;

  }; // class Rate

} // namespace utilities
  } // ns ikf

#endif // UTILS_RATE_HPP
