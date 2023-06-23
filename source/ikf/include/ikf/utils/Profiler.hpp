/******************************************************************************
* FILENAME:     Profiler.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     23.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_PROFILER_HPP
#define IKF_PROFILER_HPP

#include <string>
#include <chrono>
#include <iostream>

namespace ikf {
namespace utils
{
struct Profiler
{
  std::string                                    name;
  std::chrono::high_resolution_clock::time_point start_t;
  bool                                           mbVerbose = false;

  Profiler(std::string const& n = "", bool const verbose = false)
      : name(n), start_t(std::chrono::high_resolution_clock::now()), mbVerbose(verbose)
  {

  }

  ~Profiler()
  {
    if(mbVerbose)
    {
      using dura = std::chrono::duration<double>;
      auto d = std::chrono::high_resolution_clock::now() - start_t;
      std::cout << name << ": " << std::chrono::duration_cast<dura>(d).count() << " [sec]" << std::endl;
    }
  }

  void start() {
    start_t = std::chrono::high_resolution_clock::now();
  }

  double elapsedSec()
  {
    using dura = std::chrono::duration<double>;
    std::chrono::high_resolution_clock::time_point end       = std::chrono::high_resolution_clock::now();
    dura                                           time_span = std::chrono::duration_cast<dura>(end - start_t);
    return time_span.count();
  }

  double elapsedMS()
  {
    return elapsedSec() * 1000;
  }
};

} // namespace utils
} // ns ikf

#endif // PROFILER_HPP
