/******************************************************************************
* FILENAME:     CyclicThread.cpp
* PURPOSE:      CyclicThread
* AUTHOR:       jungr - Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     14.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/utils/CyclicThread.hpp>
#include <iostream>

namespace ikf
{
namespace utils
{
  CyclicThread::CyclicThread(const double rate_Hz) : mRate(rate_Hz)
  {
    mbShutdown = false;
    stop();
    mAVG_ms  = 0;
    mpThread = (new std::thread(&CyclicThread::threadFunc, this));
    std::cout << "Thread needs to be resumed!" << std::endl;
  }

  CyclicThread::~CyclicThread()
  {
    terminate();
    mpThread->join();
    std::cout << "thread killed!" << std::endl;
    delete mpThread;
  }

  void CyclicThread::resume()
  {
    mbRun = true;
  }

  void CyclicThread::stop()
  {
    mbRun = false;
  }

  void CyclicThread::terminate()
  {
    stop();
    mbShutdown = true;
    shutdown_();
  }

  void CyclicThread::setRateHz(const double hz)
  {
    mRate.setRate(hz);
  }

  float CyclicThread::getProcessTime_ms()
  {
    return mAVG_ms;
  }

  float CyclicThread::getProcessTime_s()
  {
    return mAVG_ms * 0.001f;
  }

  void CyclicThread::shutdown_()
  {

  }

  void CyclicThread::threadFunc()
  {
    while(!mbShutdown)
    {
      if(mbRun)
      {
        run_();
        calcAvgTime();
      }

      mRate.sleep();
    }

    std::cout << "ThreadFunc terminated..." << std::endl;
  }

  void CyclicThread::calcAvgTime()
  {
    int32_t const dur_ms = mRate.getProcessTimeMS();

    if(mAVG_ms != 0.0f)
    {
      mAVG_ms = (mAVG_ms + dur_ms) / 2;
    }
    else
    {
      mAVG_ms = dur_ms;
    }
  }

} // namespace utils
} // ns ikf
