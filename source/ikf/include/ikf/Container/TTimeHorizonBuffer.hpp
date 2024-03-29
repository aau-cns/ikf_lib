/******************************************************************************
* FILENAME:     TTimeHorizonBuffer.hpp
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
#ifndef IKF_TTIMEHORIZONBUFFER_HPP
#define IKF_TTIMEHORIZONBUFFER_HPP
#include <iostream>
#include <ikf/ikf_api.h>
#include <ikf/Container/THistoryBuffer.hpp>


namespace ikf
{
  template<typename T, class TBuffer=THistoryBuffer<T>>
  class IKF_API TTimeHorizonBuffer: public TBuffer
  {
    public:
      TTimeHorizonBuffer(double const horizon) : max_horizon_(std::abs(horizon)) {}
      ~TTimeHorizonBuffer() = default;

      TTimeHorizonBuffer<T, TBuffer> clone() {
        TTimeHorizonBuffer<T, TBuffer> buf = TTimeHorizonBuffer<T, TBuffer>(max_horizon_);
        buf.set(this->buffer_);
        return buf;
      }

      void set_horizon(const double horizon)
      {
        max_horizon_ = (std::abs(horizon));
      }

      double horizon() const
      {
        if (this->size()  > 1) {
          Timestamp oldest_t, latest_t;
          this->get_oldest_t(oldest_t);
          this->get_latest_t(latest_t);
          return latest_t.to_sec() - oldest_t.to_sec();
        }
        return 0.0;
      }

      void check_horizon()
      {
        if (this->size() > 0 && (horizon() > max_horizon_)) {
          Timestamp latest_t;
          this->get_latest_t(latest_t);
          Timestamp oldest_t(latest_t.to_sec() - max_horizon_);
          this->remove_before_t(oldest_t);
        }
      }

      void check_horizon_from_t(Timestamp const& t) {
        if (this->size() > 0) {
          Timestamp oldest_t(t.to_sec() - max_horizon_);
          this->remove_before_t(oldest_t);
        }
      }

      void check_horizon_restricted(size_t const n = 2) {
        if (this->size() > n && (horizon() > max_horizon_)) {
          Timestamp latest_t;
          this->get_latest_t(latest_t);
          Timestamp oldest_t(latest_t.to_sec() - max_horizon_);
          this->remove_before_t(oldest_t);
        }
      }

      double max_horizon() const
      {
        return max_horizon_;
      }
  protected:
      double max_horizon_;
  };
}

#endif // MMSF_TTIMEHORIZONBUFFER_HPP
