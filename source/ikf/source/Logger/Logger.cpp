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
#include <ikf/utils/IO.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <sstream>  // std::stringstream, std::stringbuf

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
    // https://stackoverflow.com/a/27678121
    const auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();

    typedef std::chrono::duration<int, std::ratio_multiply<std::chrono::hours::period, std::ratio<8> >::type>
      Days; /* UTC: +8:00 */

    Days days = std::chrono::duration_cast<Days>(duration);
    duration -= days;
    auto hours = std::chrono::duration_cast<std::chrono::hours>(duration);
    duration -= hours;
    auto minutes = std::chrono::duration_cast<std::chrono::minutes>(duration);
    duration -= minutes;
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    duration -= seconds;
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
    duration -= milliseconds;
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration);
    duration -= microseconds;

    std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    std::tm now_tm = *std::localtime(&now_c);
    std::stringstream ss;
    size_t i = 0;
    do {
      ss.str(std::string());
      ss.clear();

      ss << "/tmp/logs/ikf_log"
         << "" << i++ << "_" << now_tm.tm_year << "-" << now_tm.tm_mon << "-" << now_tm.tm_mday << "_" << now_tm.tm_hour
         << "h:" << now_tm.tm_min << "m:" << now_tm.tm_sec << "s:" << milliseconds.count()
         << "ms:" << microseconds.count() << "us.txt";
    } while (utils::IO::fileExists(ss.str()));

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(ss.str(), false);
    file_sink->set_level(spdlog::level::trace);

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(console_sink);
    sinks.push_back(file_sink);

    logger_ptr = std::make_shared<spdlog::logger>(ikf_logger_name(), std::begin(sinks), std::end(sinks));

    // force a flush when errors are detected:
    logger_ptr->flush_on(spdlog::level::warn);

    // IMPORTANT: set the logger level to the lowest level of the sinks!
    logger_ptr->set_level(spdlog::level::trace);
    logger_ptr->warn("log file created: " + ss.str());
    logger_ptr->flush();
    spdlog::register_logger(logger_ptr);
    logger_ptr->warn("logger registered... ");
  }
  return logger_ptr;
}

bool Logger::set_level(const size_t level) {
  if(level <= spdlog::level::off) {
    auto l_ptr = ikf_logger();

    l_ptr->set_level((spdlog::level::level_enum)(level) );
    return true;
  }
  return false;
}

void Logger::disable() {
  ikf_logger()->set_level(spdlog::level::off);
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
