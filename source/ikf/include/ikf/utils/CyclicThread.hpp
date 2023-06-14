/******************************************************************************
* FILENAME:     CyclicThread.hpp
* PURPOSE:      CyclicThread
* AUTHOR:       jungr - Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     14.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_UTILS_CYCLICTHREAD_HPP
#define IKF_UTILS_CYCLICTHREAD_HPP
#include <ikf/ikf_api.h>
#include <thread>
#include <memory>
#include <atomic>
#include <ikf/utils/Rate.hpp>

namespace  ikf {

namespace utils
{

  class IKF_API CyclicThread
  {
    public:
    CyclicThread(double const rate_Hz = 10);
    virtual ~CyclicThread();

    void  resume();
    void  stop();
    void  terminate();
    void  setRateHz(double const hz);
    float getProcessTime_ms();
    float getProcessTime_s();

    protected:
    virtual void run_() = 0;
    virtual void shutdown_();

    private:
    void threadFunc();
    void calcAvgTime();

    std::thread        *mpThread;
    std::atomic<bool>  mbShutdown;
    std::atomic<bool>  mbRun;
    Rate               mRate;
    std::atomic<float> mAVG_ms;
  };

} // namespace utilities
}
#endif // UTILS_CYCLICTHREAD_HPP
