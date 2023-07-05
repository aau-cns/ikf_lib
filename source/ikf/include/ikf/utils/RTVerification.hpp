/******************************************************************************
* FILENAME:     RTVerification.hpp
* PURPOSE:
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
*  Copyright (C)
*  All rights reserved.
******************************************************************************/
#ifndef IKF_RTVERIFICATION_HPP
#define IKF_RTVERIFICATION_HPP
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <iomanip>

#define RTVERIFICATION_USE_LOGGER 1

#if RTVERIFICATION_USE_LOGGER
  #include <ikf/Logger/Logger.hpp>
#endif

namespace ikf {
namespace utils
{
  class RTV
  {
    public:

    static bool Verify(bool condition,
                       const char *expression,
                       const char *msg,
                       bool expected,
                       const char *file,
                       int line_num,
                       bool bFail = false)
    {
      if(condition != expected)
      {
        std::stringstream ss;
        ss << "Error: " << expression << " is not " << std::boolalpha << expected << "! " << file << ": " << line_num
            << "\n";

        if(msg)
        {
          ss << "--> msg:'" << msg << "'\n";
        }

        if(bFail)
        {
#if RTVERIFICATION_USE_LOGGER
          Logger::ikf_logger()->error(ss.str());
#endif // RTVERIFICATION_USE_LOGGER
          throw std::runtime_error(ss.str());
        }
        else
        {
#if RTVERIFICATION_USE_LOGGER
          Logger::ikf_logger()->error(ss.str());
#else
          std::cout << ss.str() << std::endl;
#endif // RTVERIFICATION_USE_LOGGER
          return false;
        }
      }

      return true;
    }
    static bool Verify(bool condition,
                       const char *expression,
                       std::string const&  msg,
                       bool expected,
                       const char *file,
                       int line_num,
                       bool bFail = false)
    {
      if(condition != expected)
      {
        std::stringstream ss;
        ss << "Error: " << expression << " is not " << std::boolalpha << expected << "! " << file << ": " << line_num
           << "\n";

        if(!msg.empty())
        {
          ss << "--> msg:'" << msg << "'\n";
        }

        if(bFail)
        {
#if RTVERIFICATION_USE_LOGGER
          Logger::ikf_logger()->error(ss.str());
#endif // RTVERIFICATION_USE_LOGGER
          throw std::runtime_error(ss.str());
        }
        else
        {
#if RTVERIFICATION_USE_LOGGER
          Logger::ikf_logger()->error(ss.str());
#else
          std::cout << ss.str() << std::endl;
#endif // RTVERIFICATION_USE_LOGGER
          return false;
        }
      }

      return true;
    }

  }; // class RTV

} // namespace utils



// only verification of condition
#define RTV_EXPECT_TRUE(condition) \
  utils::RTV::Verify(condition, #condition, 0, true, __FILE__, __LINE__)

#define RTV_EXPECT_FALSE(condition) \
utils::RTV::Verify(condition, #condition, 0, false, __FILE__, __LINE__)

#define RTV_EXPECT_TRUE_MSG(condition, msg) \
  utils::RTV::Verify(condition, #condition, msg, true, __FILE__, __LINE__)

#define RTV_EXPECT_FALSE_MSG(condition, msg) \
utils::RTV::Verify(condition, #condition, msg, false, __FILE__, __LINE__)

// throws exception if invalid
#define RTV_EXPECT_TRUE_THROW(condition, msg) \
  utils::RTV::Verify(condition, #condition, msg, true, __FILE__, __LINE__, true)

// returns the result of the verification
#define RTV_EXPECT_TRUE_RET(condition, msg) \
  return utils::RTV::Verify(condition, #condition, msg, true, __FILE__, __LINE__);

#define RTV_EXPECT_TRUE_RET_(condition) \
  return utils::RTV::Verify(condition, #condition, 0, true, __FILE__, __LINE__);

// returns false if invalid
#define RTV_EXPECT_TRUE_CONDRET(condition, msg) \
  if(!utils::RTV::Verify(condition, #condition, msg, true, __FILE__, __LINE__)) return false;


#define RTV_EXPECT_TRUE_CONDRET_(condition) \
  if(!utils::RTV::Verify(condition, #condition, 0, true, __FILE__, __LINE__)) return false;

#define RTV_EXPECT_TRUE_CONDRET2_(condition) \
  if(!utils::RTV::Verify(condition, #condition, 0, true, __FILE__, __LINE__)) return;

} // namespace ikf
#endif // RTVERIFICATION_H
