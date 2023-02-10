/******************************************************************************
* FILENAME:     Timestamp.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     29.05.2018
*
*  Copyright (C) 2018
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <ikf/container/Timestamp.hpp>
#include <cmath>
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
      throw std::runtime_error("Time is out of dual 32-bit range");
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

  Timestamped::~Timestamped() {}

  Timestamp Timestamped::get_timestamp()
  {
    return static_cast<Timestamp>(*this);
  }

  bool Timestamped::operator <(const Timestamped &rhs) const
  {
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
