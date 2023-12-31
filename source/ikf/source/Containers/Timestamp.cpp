/******************************************************************************
* FILENAME:     Timestamp.cpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         <roland.jung@ieee.org>
* VERSION:      v0.0.0
* CREATION:     29.05.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <cmath>
#include <ikf/Container/Timestamp.hpp>
#include <ikf/Logger/Logger.hpp>
#include <limits>
#include <stdexcept>

namespace ikf
{
  Timestamp::Timestamp(): sec(0), nsec(0) {}


  Timestamp::Timestamp(int32_t sec_, int32_t nsec_): sec(sec_), nsec(nsec_) {}

  Timestamp::Timestamp(const double t_)
  {
    from_sec(t_);
  }

  Timestamp::Timestamp(const int64_t stamp_ns) {
    from_stamp_ns(stamp_ns);
  }


  Timestamp::~Timestamp() {}

  double Timestamp::to_sec() const
  {
    return (double)sec + 1e-9*(double)nsec;
  }

  void Timestamp::from_sec(double const t)
  {
    int64_t sec64 = (int64_t)std::floor(t);
    if (sec64 > INT32_MAX)
    {
      ikf::Logger::ikf_logger()->error("Time is out of dual 32-bit range" + std::to_string(t));
      sec64 = 0;
    }
    sec = (int32_t)sec64;
    nsec = (int32_t)std::round((t-sec) * 1e9);
    // avoid rounding errors
    sec += (nsec / 1000000000ul);
    nsec %= 1000000000ul;
  }

  bool Timestamp::is_zero() const {
    return (sec == 0) && (nsec == 0);
  }

  int64_t Timestamp::stamp_ns() const {
    std::int64_t time64 = static_cast<std::int64_t>(sec) * 1000000000ul;
    time64 = time64 + nsec;
    return time64;
  }

  void Timestamp::from_stamp_ns(const int64_t stamp_ns) {
    sec = stamp_ns / 1000000000ul;
    nsec = stamp_ns % 1000000000ul;
  }

  std::string Timestamp::str() const { return std::to_string(to_sec()); }

  bool Timestamp::operator<(const Timestamp &rhs) const {
    if (sec < rhs.sec) {
      return true;
    } else if (sec == rhs.sec) {
      if (nsec < rhs.nsec) {
        return true;
      }
    }
    return false;
  }

  bool Timestamp::operator==(const Timestamp &rhs) const { return (sec == rhs.sec) && (nsec == rhs.nsec); }

  bool Timestamp::operator<=(const Timestamp &rhs) const { return !(*this > rhs); }

  bool Timestamp::operator>(const Timestamp &rhs) const { return !(*this == rhs) && !(*this < rhs); }

  Timestamp &Timestamp::operator-=(const Timestamp &rhs) {
    from_sec(to_sec() - rhs.to_sec());
    return *this;
  }

  Timestamp &Timestamp::operator+=(const Timestamp &rhs) {
    from_sec(to_sec() + rhs.to_sec());
    return *this;
  }

  Timestamped::~Timestamped() {}

  Timestamp Timestamped::get_timestamp() { return static_cast<Timestamp>(*this); }

  bool Timestamped::operator<(const Timestamped &rhs) const {
    if(sec < rhs.sec)
    {
      return true;
    }
    else if(sec == rhs.sec)
    {
      if(nsec < rhs.nsec)
      {
        return true;
      }
    }
    return false;
  }

  bool Timestamped::operator==(const Timestamped &rhs) const
  {
    return (sec == rhs.sec) && (nsec == rhs.nsec);
  }

  bool Timestamped::operator >(const Timestamped &rhs) const
  {
    return !(*this == rhs ) && !(*this < rhs);
  }

} // namespace ikf
