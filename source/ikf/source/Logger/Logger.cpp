/******************************************************************************
* FILENAME:     Logger.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     04.07.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/Logger/Logger.hpp>
#include <sstream>      // std::stringstream, std::stringbuf
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace ikf {

std::string Logger::ikf_logger_name() {
  return "ikf_log";
}

std::shared_ptr<spdlog::logger> Logger::ikf_logger() {
  auto logger_ptr = spdlog::get(ikf_logger_name());
  if(!logger_ptr)
  {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::warn);
    console_sink->set_pattern("[ikf_log] [%^%l%$] %v");

    // TODO: maybe use dedicated thirdparty lib: https://github.com/HowardHinnant/date
    std::time_t now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm now_tm = *std::localtime(&now_c);
    std::stringstream ss;
    ss << "logs/ikf_log_" << now_tm.tm_year << "-" << now_tm.tm_mon << "-" <<  now_tm.tm_mday << "-" <<  now_tm.tm_hour << "-" <<  now_tm.tm_min << "-" <<  now_tm.tm_sec << ".txt";
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(ss.str(), false);
    file_sink->set_level(spdlog::level::trace);

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(console_sink);
    sinks.push_back(file_sink);

    logger_ptr = std::make_shared<spdlog::logger>(ikf_logger_name(), std::begin(sinks), std::end(sinks));

    // force a flush when errors are detected:
    logger_ptr->flush_on(spdlog::level::err);

    // IMPORTANT: set the logger level to the lowest level of the sinks!
    logger_ptr->set_level(spdlog::level::trace);

    spdlog::register_logger(logger_ptr);
  }
  return logger_ptr;
}

std::shared_ptr<spdlog::logger> Logger::setup_logger(std::vector<spdlog::sink_ptr> sinks)
{
  if(sinks.size() > 0)
  {
    auto logger_ptr= spdlog::get(ikf_logger_name());
    if(logger_ptr) {
      logger_ptr->flush();
      spdlog::drop(ikf_logger_name());
    }

    logger_ptr = std::make_shared<spdlog::logger>(ikf_logger_name(),
                                                std::begin(sinks),
                                                std::end(sinks));

    // force a flush when errors are detected:
    logger_ptr->flush_on(spdlog::level::err);

    // IMPORTANT: set the logger level to the lowest level of the sinks!
    logger_ptr->set_level(spdlog::level::trace);
    spdlog::register_logger(logger_ptr);
    return logger_ptr;
  } else {
    auto logger_ptr = spdlog::get(ikf_logger_name());
    if(!logger_ptr) {
      logger_ptr = ikf_logger();
    }
    return logger_ptr;
  }
}

void Logger::log_trace(std::string message) {
  auto logger = spdlog::get(ikf_logger_name());
  if(logger)
  {
    logger->trace("{}::{}", __FUNCTION__, message);
  }
}

void Logger::log_debug(std::string message)
{
  auto logger = spdlog::get(ikf_logger_name());
  if(logger)
  {
    logger->debug("{}::{}", __FUNCTION__, message);
  }
}

void Logger::log_info(std::string message) {
  auto logger = spdlog::get(ikf_logger_name());
  if(logger)
  {
    logger->info("{}::{}", __FUNCTION__, message);
  }
}
void Logger::log_warn(std::string message) {
  auto logger = spdlog::get(ikf_logger_name());
  if(logger)
  {
    logger->warn("{}::{}", __FUNCTION__, message);
  }
}




}
