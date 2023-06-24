/******************************************************************************
* FILENAME:     ProgressBar.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     24.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/utils/ProgressBar.hpp>
namespace ikf {
namespace utils {

ProgressBar::ProgressBar(size_t numSteps, size_t barCharLen, size_t updStepPerc, std::string startMsg, std::string endMsg, bool showremTime, bool showBar, bool showPercentage, bool showActualNum, bool showFinalTime, char barCharSymbol, char emptybarCharSymbol, bool createNewLine) :
    numSteps(numSteps),
    barCharLen(barCharLen), updStep(updStepPerc), startMsg(startMsg), endMsg(endMsg),
    showremTime(showremTime), showBar(showBar), showPercentage(showPercentage),
    showActualNum(showActualNum), showFinalTime(showFinalTime), barCharSymbol(barCharSymbol),
    emptybarCharSymbol(emptybarCharSymbol), createNewLine(createNewLine)
{
  m_profiler.start();

  updStep = std::min((size_t)100, std::max(updStepPerc, (size_t)0));
  increment = std::ceil((numSteps * 100)/updStep)/100;

  nextRenderPoint = increment;

}

void ProgressBar::progress() {
  tick = std::min(tick +1, numSteps);
  update(tick);
}

void ProgressBar::update(const size_t i) {
  if(i < nextRenderPoint) {
    return;
  }
  if(i > 0) {
    if(createNewLine)
    {
      std::cout << std::endl;
    }
  }
  nextRenderPoint = std::min(nextRenderPoint + increment, numSteps);




  if(i >= numSteps) {
    // Done
    std::cout << "[" << endMsg ;
    if(showFinalTime) {
      if(final_time_sec == 0.0) {
        final_time_sec = m_profiler.elapsedSec();
      }
      std::cout << " after " << sec_to_HHMMSS(final_time_sec) << " ";
    }
    std::cout << "]"<< std::endl;
  } else {
    // in progress:
    size_t barsToPrint = floor((i * barCharLen) / numSteps);
    std::string  bar = "[" + std::string(barsToPrint, barCharSymbol) +  std::string(barCharLen-barsToPrint, emptybarCharSymbol) + "]";

    std::cout << bar;
    if(showActualNum) {
      std::cout << " " << std::to_string(i) << "/" << std::to_string(numSteps);
    }

    double percent = ((i * 100.0) / numSteps);
    if(showPercentage) {
      std::cout << " " << percent << "%";
    }
    if(showremTime) {
      double curr_sec = m_profiler.elapsedSec();
      double rem_sec = curr_sec *(100-percent);
      std::cout << "[" << sec_to_HHMMSS(curr_sec) << "/" << sec_to_HHMMSS(rem_sec) << "]";
    }
    std::cout << std::endl;
  }

}

std::string ProgressBar::sec_to_HHMMSS(const double s) {
  int sec = std::floor(s);
  int hour =(sec / 3600);
  int min = (sec % 3600) / 60;
  sec = sec % 60;
  return std::to_string(hour) + ":" + std::to_string(min) + ":" + std::to_string(sec);
}


} // ns utils
} // ns ikf
