/******************************************************************************
* FILENAME:     ProgressBar.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     24.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_PROGRESSBAR_HPP
#define IKF_PROGRESSBAR_HPP
#include <ikf/ikf_api.h>
#include <string>
#include <chrono>
#include <iostream>
#include <cmath>

#include <ikf/utils/Profiler.hpp>

namespace ikf {
namespace utils {

class IKF_API ProgressBar {
public:
  ProgressBar(size_t numSteps,
              size_t barCharLen = 20,
              size_t updStepPerc = 10,
              std::string startMsg = "Completed ",
              std::string endMsg = " Done.",
              bool showremTime = true,
              bool showBar = true,
              bool showPercentage = true,
              bool showActualNum = true,
              bool showFinalTime = true,
              char barCharSymbol = '=',
              char emptybarCharSymbol = ' ',
              bool createNewLine = false);

  void progress();

  void update(size_t const i);

  std::string sec_to_HHMMSS(double const s);


private:
  size_t tick = 0;
  size_t numSteps = 0;
  size_t nextRenderPoint = 0;
  double final_time_sec = 0.0;


  size_t increment = 0;
  size_t barCharLen = 20;
  size_t updStep = 10;
  std::string startMsg = "Completed ";
  std::string endMsg = " Done.";
  bool showremTime = true;
  bool showBar = true;
  bool showPercentage = true;
  bool showActualNum = false;
  bool showFinalTime = true;
  char barCharSymbol = '=';
  char emptybarCharSymbol = ' ';
  bool createNewLine = false;
  Profiler m_profiler;
};

}
}

#endif // PROGRESSBAR_HPP
