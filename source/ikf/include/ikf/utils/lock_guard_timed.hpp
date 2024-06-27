/******************************************************************************
 * FILENAME:     lock_guard_timed.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     27.06.2024
 *
 *  Copyright (C) 2024
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#ifndef IKF_LOCK_GUARD_TIMED_HPP
#define IKF_LOCK_GUARD_TIMED_HPP
#include <ikf/ikf_api.h>

#include <chrono>
#include <mutex>

namespace ikf {
/** @brief A simple scoped lock type.
 *
 * A lock_guard controls mutex ownership within a scope, releasing
 * ownership in the destructor.
 */
template <typename _Mutex>
class IKF_API lock_guard_timed {
public:
  typedef _Mutex mutex_type;

  explicit lock_guard_timed(mutex_type& __m, std::chrono::milliseconds __timeout_ms)
    : _M_device(__m), _mtx_timeout_ms(__timeout_ms) {}

  bool try_lock() { return _M_device.try_lock_for(_mtx_timeout_ms); }
  ~lock_guard_timed() { _M_device.unlock(); }

  lock_guard_timed(const lock_guard_timed&) = delete;
  lock_guard_timed& operator=(const lock_guard_timed&) = delete;

private:
  mutex_type& _M_device;
  std::chrono::milliseconds _mtx_timeout_ms;
};
}  // namespace ikf

#endif // LOCK_GUARD_TIMED_HPP
