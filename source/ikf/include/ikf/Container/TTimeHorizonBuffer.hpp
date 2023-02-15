/******************************************************************************
* FILENAME:     TTimeHorizonBuffer.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       Roland Jung
* MAIL:         jungr-ait@github
* VERSION:      v0.0.0
* CREATION:     19.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
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
  protected:
      double max_horizon_;
  };
}

#endif // MMSF_TTIMEHORIZONBUFFER_HPP
