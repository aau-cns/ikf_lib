/******************************************************************************
* FILENAME:     Rate.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     14.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/utils/Rate.hpp>

namespace ikf
{
namespace utils
{

  Rate::Rate(const double& hz)
  {
    setRate(hz);
    mtStart = Clock::now();
  }

  void Rate::sleep()
  {
    mtStop       = Clock::now();
    mProcessTime = std::chrono::duration_cast<std::chrono::milliseconds>(mtStop - mtStart);
    std::int32_t sleep_ms = mSleep_ms - mProcessTime.count();

    if(sleep_ms > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }

    mtStart = Clock::now();
  }

  int32_t Rate::getProcessTimeMS() const
  {
    return mProcessTime.count();
  }

  void Rate::setRate(const double hz)
  {
    mSleep_ms = static_cast<std::uint32_t>((hz == 0) ? 0 : 1000 / hz);
  }

} // namespace utils
}
