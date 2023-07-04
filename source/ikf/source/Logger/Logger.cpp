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
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace ikf {

std::string Logger::ikf_logger_name() {
  return "ikf_log";
}

std::shared_ptr<spdlog::logger> Logger::ikf_logger() {
  auto logger = spdlog::get(ikf_logger_name());
  if(!logger)
  {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::warn);
    console_sink->set_pattern("[ikf_log] [%^%l%$] %v");

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/ikf_log.txt", true);
    file_sink->set_level(spdlog::level::trace);

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(console_sink);
    sinks.push_back(file_sink);

    logger = std::make_shared<spdlog::logger>(ikf_logger_name(), std::begin(sinks), std::end(sinks));
    logger->flush_on(spdlog::level::err);
    logger->set_level(spdlog::level::trace);
  }
  return logger;
}

std::shared_ptr<spdlog::logger> Logger::setup_logger(std::vector<spdlog::sink_ptr> sinks)
{
  auto logger = spdlog::get(ikf_logger_name());
  if(!logger)
  {
    if(sinks.size() > 0)
    {
      logger = std::make_shared<spdlog::logger>(ikf_logger_name(),
                                                std::begin(sinks),
                                                std::end(sinks));
    }
    else
    {
      logger = ikf_logger();
    }
    spdlog::register_logger(logger);
  }
  else if(sinks.size()) {
    spdlog::drop(ikf_logger_name());
    logger = std::make_shared<spdlog::logger>(ikf_logger_name(),
                                              std::begin(sinks),
                                              std::end(sinks));
    spdlog::register_logger(logger);
  }

  return logger;
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
