/******************************************************************************
* FILENAME:     Logger.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     04.07.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_LOGGER_HPP
#define IKF_LOGGER_HPP
#include <ikf/ikf_api.h>
#include <memory>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/logger.h>

namespace ikf
{

class IKF_API Logger {
public:

  // logs everyting (trace) to a file, logs warnings+ to the console
  static std::string ikf_logger_name();

  // logs everyting (trace) to a file, logs warnings+ to the console; logger level by default is trace (all)
  static std::shared_ptr<spdlog::logger> ikf_logger();
  static bool set_level(size_t const level);
  static void disable();

  // creates and registeres a new logger given the sinks
  static std::shared_ptr<spdlog::logger> setup_logger(std::vector<spdlog::sink_ptr> sinks);

  void static log_trace(std::string message);
  void static log_debug(std::string message);
  void static log_info(std::string message);
  void static log_warn(std::string message);
};

}


#endif // LOGGER_HPP
