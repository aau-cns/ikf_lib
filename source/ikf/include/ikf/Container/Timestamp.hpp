/******************************************************************************
 * FILENAME:     Timestamp.hpp
 * PURPOSE:      Part of the ikf_lib
 * AUTHOR:       Roland Jung
 * MAIL:         jungr-ait@github
 * VERSION:      v0.0.0
 * CREATION:     19.01.2023
 *
 * Copyright (C) 2023 Roland Jung, Control of Networked Systems, University of Klagenfurt, Austria.
 *
 * All rights reserved.
 *
 * This software is licensed under the terms of the BSD-2-Clause-License with
 * no commercial use allowed, the full terms of which are made available
 * in the LICENSE file. No license in patents is granted.
 *
 * You can contact the author at <roland.jung@aau.at>
 ******************************************************************************/
#ifndef IKF_TIMESTAMP_HPP
#define IKF_TIMESTAMP_HPP

#include <cstdint>
#include <ikf/ikf_api.h>
#include <ostream>

namespace ikf {
///
/// \brief The Timestamp class
/// Q1: Why not using a floating point represtation?
/// A1: Floats do not support a unique data representation (huge range dynamics, rounding and machine precission might
/// lead to data comparision issues). The current implementation allows a discretization up to nanoseconds (should be
/// sufficient for most applications) and is inspired by ROS.

/**
 * @brief The Timestamp class
 * The current time is roughly 1720712106 seconds since the Epoch (1970-01-01 00:00 UTC) which requires already
 * 31-bit. Since we use a signed integer, this time would fit in INT32_t variable, but leaves no headroom for numerical
 * operations (adding two of them to compute the average causes an overflow!).
 * Using INT32_t for seconds, the max positive values is 2147483647 which is Tuesday, January 19, 2038 3:14:07 AM,
 * according to https://www.epochconverter.com/.
 * Therefore the seconds are represented as * INT64_t.
 */
class IKF_API Timestamp {
public:
  std::int64_t sec = 0;
  std::int32_t nsec = 0;

  ///
  /// \brief Timestamp: CTORs
  ///
  Timestamp();
  Timestamp(std::int64_t sec_, std::int32_t nsec_);
  Timestamp(double const t_);
  Timestamp(std::int64_t const stamp_ns);
  virtual ~Timestamp() = default;

  double to_sec() const;
  void from_sec(double const t = 0.0);
  bool is_zero() const;

  std::int64_t stamp_ms() const;
  void from_stamp_ms(std::int64_t const stamp_ms = 0);
  std::int64_t stamp_us() const;
  void from_stamp_us(std::int64_t const stamp_us = 0);
  //  max: +9223372036.854775807 [s] == Friday, April 11, 2262 11:47:16 PM
  std::int64_t stamp_ns() const;
  void from_stamp_ns(std::int64_t const stamp_ns = 0);

  std::string str() const;

  friend std::ostream& operator<<(std::ostream& out, const Timestamp& obj) {
    out << obj.str();
    return out;
  }

  bool operator<(Timestamp const& rhs) const;
  bool operator==(Timestamp const& rhs) const;
  bool operator<=(Timestamp const& rhs) const;
  bool operator>(Timestamp const& rhs) const;
  Timestamp operator+(const Timestamp& rhs) const;
  Timestamp operator-(const Timestamp& rhs) const;
  Timestamp& operator-=(const Timestamp& rhs);
  Timestamp& operator+=(const Timestamp& rhs);
};

class IKF_API Timestamped : public Timestamp {
public:
  virtual ~Timestamped();
  virtual ikf::Timestamp get_timestamp();

  bool operator<(Timestamped const& rhs) const;

  bool operator==(Timestamped const& rhs) const;

  bool operator>(Timestamped const& rhs) const;

};  // Timestamped

}  // namespace ikf

#endif  // IKF_TIMESTAMP_HPP
