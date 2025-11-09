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

namespace ikf {
Timestamp::Timestamp() : sec(0), nsec(0) {}

Timestamp::Timestamp(int64_t sec_, int32_t nsec_) : sec(sec_), nsec(nsec_) {}

Timestamp::Timestamp(const double t_) { from_sec(t_); }

Timestamp::Timestamp(const int64_t stamp_ns) { from_stamp_ns(stamp_ns); }


double Timestamp::to_sec() const { return (double)sec + 1e-9 * (double)nsec; }

void Timestamp::from_sec(double const t) {
  if(std::isfinite(t) && !std::isnan(t)) {
    int64_t sec64 = (int64_t)std::floor(t);
    if (sec64 <= __INT64_C(0x1FFFFFFFF)) {
      sec = (int64_t)sec64;
      nsec = (int32_t)std::round((t - sec) * 1e9);
      // avoid rounding errors
      sec += (nsec / 1000000000l);
      nsec %= 1000000000l;
    } else {
      ikf::Logger::ikf_logger()->error(" Timestamp::from_sec(): Time is out of 33-bit range: " + std::to_string(t));
      sec = 0;
      nsec = 0;
    }
  } else {
    // silently set it to zero!
    sec = 0;
    nsec = 0;
  }
}

bool Timestamp::is_zero() const { return (sec == 0) && (nsec == 0); }

int64_t Timestamp::stamp_ms() const {
  std::int64_t time64 = static_cast<std::int64_t>(sec) * 1000l;
  time64 = time64 + nsec / 1000000l;
  return time64;
}

void Timestamp::from_stamp_ms(const int64_t stamp_ms) {
  sec = stamp_ms / 1000;
  nsec = (stamp_ms % 1000) * 1000000l;
}

int64_t Timestamp::stamp_us() const { return sec * 1000000l + nsec / 1000l; }

void Timestamp::from_stamp_us(const int64_t stamp_us) {
  sec = stamp_us / 1000000l;
  nsec = (stamp_us % 1000000l) * 1000ul;
}

int64_t Timestamp::stamp_ns() const {
  // the seconds parts can take positive 33bits, cause we need 30bits for the nano seconds
  if (sec <= __INT64_C(9223372036)) {
    std::int64_t time64 = static_cast<std::int64_t>(sec) * 1000000000l;
    time64 = time64 + nsec;
    return time64;
  } else {
    // seconds exceed: Friday, April 11, 2262 11:47:16 PM!
    ikf::Logger::ikf_logger()->error("Timestamp::stamp_ns(): time is out of convertible range! ");
    return 0;
  }
}

void Timestamp::from_stamp_ns(const int64_t stamp_ns) {
  sec = stamp_ns / 1000000000l;
  nsec = stamp_ns % 1000000000l;
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

Timestamp Timestamp::operator+(const Timestamp &rhs) const { return Timestamp(to_sec() + rhs.to_sec()); }

Timestamp Timestamp::operator-(const Timestamp &rhs) const { return Timestamp(to_sec() - rhs.to_sec()); }

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
  if (sec < rhs.sec) {
    return true;
  } else if (sec == rhs.sec) {
    if (nsec < rhs.nsec) {
      return true;
    }
  }
  return false;
}

bool Timestamped::operator==(const Timestamped &rhs) const { return (sec == rhs.sec) && (nsec == rhs.nsec); }

bool Timestamped::operator>(const Timestamped &rhs) const { return !(*this == rhs) && !(*this < rhs); }

}  // namespace ikf
