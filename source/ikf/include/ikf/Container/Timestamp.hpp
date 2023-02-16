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

#include <ikf/ikf_api.h>
#include <cstdint>
#include <ostream>

namespace ikf
{
///
/// \brief The Timestamp class
/// Q1: Why not using a floating point represtation?
/// A1: Floats do not support a unique data representation (huge range dynamics, rounding and machine precission might lead to data comparision issues). The current implementation allows a discretization up to nanoseconds (should be sufficient for most applications) and is inspired by ROS.
  class IKF_API Timestamp
  {
    public:
      std::int32_t sec = 0;
      std::int32_t nsec = 0;

      ///
      /// \brief Timestamp: CTORs
      ///
      Timestamp();
      Timestamp(std::int32_t sec_, std::int32_t nsec_);
      Timestamp(double const t_);
      Timestamp(std::int64_t const stamp_ns);
      virtual ~Timestamp();

      double to_sec() const;
      void from_sec(double const t);
      bool is_zero() const;

      std::int64_t stamp_ns() const;
      void from_stamp_ns(std::int64_t const stamp_ns);

      std::string str() const {
        return std::to_string(sec) + "." + std::to_string(nsec);
      }

      friend std::ostream &operator<< (std::ostream& out, const Timestamp& obj){
          out << obj.sec << "." << obj.nsec;
          return out;
      }

      bool operator < (Timestamp const& rhs)  const {
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

      bool operator==(Timestamp const& rhs) const  {
          return (sec == rhs.sec) && (nsec == rhs.nsec);
      }

      bool operator<=(Timestamp const& rhs) const  {
          return !(*this > rhs);
      }

      bool operator > (Timestamp const& rhs) const  {
          return !(*this == rhs ) && !(*this < rhs);
      }

      // TODO: operator + and -
  };

  class IKF_API Timestamped: public Timestamp
  {
    public:
      virtual ~Timestamped();
      virtual ikf::Timestamp get_timestamp();

      bool operator < (Timestamped const& rhs) const;

      bool operator==(Timestamped const& rhs) const;

      bool operator > (Timestamped const& rhs) const;


  }; // Timestamped

} // namespace ikf

#endif // IKF_TIMESTAMP_HPP
