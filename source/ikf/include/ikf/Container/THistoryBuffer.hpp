/******************************************************************************
 * FILENAME:     THistoryBuffer.hpp
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
#ifndef IKF_THISTORYBUFFER_HPP
#define IKF_THISTORYBUFFER_HPP
#include <algorithm>
#include <functional>
#include <ikf/Container/TStampedData.hpp>
#include <ikf/ikf_api.h>
#include <list>
#include <map>
#include <numeric>
#include <ostream>
#include <set>

namespace ikf {
/**
 * @brief The THistoryBuffer class to store data with associated timestamps at [ns] resolution in a INT64_T.
 *
 * The current time since epoch (1970-01-01 00:00 UTC) is roughly 1720712106 seconds which requires already
 * 31-bit. Since we use an int64_t for the timestamp in nanoseconds, the max date since epoch in is
 * +9223372036.854775807 [s] == ´Friday, April 11, 2262 11:47:16 PM
 */
template <typename T>
class IKF_API THistoryBuffer {
public:
  typedef TStampedData<T> TData;
  typedef std::map<std::int64_t, T> TContainer;

  void insert(T const& data, Timestamp const t);
  void insert(T const& data, double const t_sec);
  void insert(TData const& data);
  size_t size() const;
  bool empty() const { return buffer_.empty(); }
  bool at(size_t const idx, TData& data);
  bool at(size_t const idx, T& data);
  bool at(size_t const idx, Timestamp& t);
  void clear();

  void set(TContainer const& buffer) { buffer_ = buffer; }
  TContainer& get() { return buffer_; }

  bool index_at_t(Timestamp const& t, size_t& idx) const;

  bool exist_at_t(double const t) const;
  bool exist_at_t(Timestamp const& t) const;
  bool exist_after_t(Timestamp const& t) const;
  bool exist_before_t(Timestamp const& t) const;

  bool get_oldest_t(Timestamp& t) const;
  bool get_latest_t(Timestamp& t) const;
  bool get_oldest(T& elem) const;
  bool get_latest(T& elem) const;

  THistoryBuffer get_between_t1_t2(Timestamp const& t1, Timestamp const& t2) {
    if (t1 < t2 && size() > 0) {
      auto itlow = lower_bound(t1);
      auto itup = upper_bound(t2);
      THistoryBuffer h;
      h.set(TContainer(itlow, itup));
      return h;
    }
    return THistoryBuffer();
  }

  ///
  /// \brief op binary operation function object that will be applied. The binary operator takes the current
  ///           accumulation value a (initialized to init) and the value of the current element b.
  ///           The signature of the function should be equivalent to the following: T fun(const T &a, const T &b)
  ///
  template <typename BinaryOperation>  // Ret fun(const Type1 &a, const Type2 &b)
  inline T accumulate_between_t1_t2(Timestamp const& t1, Timestamp const& t2, T init, BinaryOperation op) const {
    if (t1 <= t2 && size() > 0) {
      auto __first = lower_bound(t1);
      auto __last = upper_bound(t2);
      for (; __first != __last; ++__first) init = op(init, __first->second);
      return init;
    }
    return init;
  }

  template <typename BinaryOperation>  // Ret fun(const Type1 &a, const Type2 &b)
  inline T accumulate(T init, BinaryOperation op) {
    auto __first = buffer_.begin();
    auto __last = buffer_.end();
    for (; __first != __last; ++__first) init = op(init, __first->second);
    return init;
  }

  template <typename Operation>
  inline void foreach_between_t1_t2(Timestamp const& t1, Timestamp const& t2, Operation op) {
    if (t1 <= t2 && size() > 0) {
      auto __first = lower_bound(t1);
      auto __last = upper_bound(t2);

      for (; __first != __last; ++__first) op(__first->second);
    }
  }

  template <typename Operation>
  inline void foreach (Operation op) {
    auto __first = buffer_.begin();
    auto __last = buffer_.end();
    for (; __first != __last; ++__first) {
      op(__first->second);
    }
  }

  template <typename Operation>
  inline void foreach_t(Operation op) {
    auto __first = buffer_.begin();
    auto __last = buffer_.end();
    for (; __first != __last; ++__first) {
      op(__first->first);
    }
  }

  template <typename Operation>
  inline void foreach_reverse(Operation op) {
    auto __first = buffer_.rbegin();
    auto __last = buffer_.rend();
    for (; __first != __last; ++__first) {
      op(__first->second);
    }
  }

  bool get_at_t(Timestamp const& t, T& elem) const;
  bool get_at_t(Timestamp const& t, TData& elem) const;
  bool get_at_t(double const t_sec, T& elem) const;
  bool get_before_t(Timestamp const& t, T& elem) const;
  bool get_before_t(Timestamp const& t, Timestamp& elem) const;
  bool get_before_t(Timestamp const& t, TData& elem) const;
  bool get_after_t(Timestamp const& t, T& elem) const;
  bool get_after_t(Timestamp const& t, Timestamp& elem) const;
  bool get_after_t(Timestamp const& t, TData& elem) const;
  bool get_closest_t(Timestamp const& t, T& elem) const;
  bool get_closest_t(Timestamp const& t, Timestamp& elem) const;
  bool get_closest_t(Timestamp const& t, TData& elem) const;
  void remove_at_t(Timestamp const& t);
  void remove_at_t(double const t);
  void remove_before_t(Timestamp const& t);
  void remove_before_t(double const t);
  void remove_after_t(Timestamp const& t);
  void remove_after_t(double const t);
  void remove_every_N(size_t const subsample);
  void subsample_by_N(size_t const subsample);

  friend std::ostream& operator<<(std::ostream& out, const THistoryBuffer<T>& obj) {
    out << "THistoryBuffer: len=" << obj.size() << std::endl;
    for (auto const& elem : obj.buffer_) {
      out << "* t=" << elem.first << ", data=" << elem.second << "\n";
    }
    return out;
  }

  void print(std::ostream& out, size_t const N = 0) {
    size_t len = N;
    if (N == 0) {
      len = size();
    }

    size_t cnt = 0;
    for (auto const& elem : buffer_) {
      if (cnt < len) {
        out << "* t=" << elem.first << ", data=" << elem.second << "\n";
      } else {
        break;
      }
      cnt++;
    }
  }

  void print_reverse(std::ostream& out, size_t const N = 0) {
    size_t len = N;
    if (N == 0) {
      len = size();
    }

    size_t cnt = 0;
    for (auto iter = buffer_.rbegin(); iter != buffer_.rend(); ++iter) {
      if (cnt < len) {
        out << "* t=" << iter->first << ", data=" << iter->second << "\n";
      } else {
        break;
      }
      cnt++;
    }
  }

protected:
  friend std::ostream;
  TContainer buffer_;

private:
  void insert_sorted(Timestamp const& t, T const& elem) { buffer_[t.stamp_ns()] = elem; }

  // https://stackoverflow.com/a/72835502
  typename TContainer::const_iterator upper_bound(Timestamp const& t) const {
    return std::upper_bound(
      buffer_.begin(), buffer_.end(), t.stamp_ns(),
      [](const std::int64_t& value, const std::pair<std::int64_t, T> iter) { return value < iter.first; });
  }

  typename TContainer::const_iterator lower_bound(Timestamp const& t) const {
    return std::lower_bound(
      buffer_.begin(), buffer_.end(), t.stamp_ns(),
      [](const std::pair<std::int64_t, T>& iter, const std::int64_t& value) { return iter.first < value; });
  }
};

template <typename T>
void THistoryBuffer<T>::insert(const T& data, const Timestamp t) {
  insert_sorted(t, data);
}

template <typename T>
void THistoryBuffer<T>::insert(const T& data, const double t_sec) {
  insert_sorted(Timestamp(t_sec), data);
}

template <typename T>
void THistoryBuffer<T>::insert(const TData& data) {
  insert_sorted(data.stamp, data.data);
}

template <typename T>
size_t THistoryBuffer<T>::size() const {
  return buffer_.size();
}

template <typename T>
bool THistoryBuffer<T>::at(const size_t idx, TData& data) {
  if (idx < buffer_.size()) {
    auto iter = buffer_.begin();
    std::advance(iter, idx);
    data.data = iter->second;
    data.stamp = Timestamp(iter->first);
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::at(const size_t idx, T& data) {
  if (idx < buffer_.size()) {
    auto iter = buffer_.begin();
    std::advance(iter, idx);
    data = iter->second;
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::at(const size_t idx, Timestamp& t) {
  if (idx < buffer_.size()) {
    auto iter = buffer_.begin();
    std::advance(iter, idx);
    t = Timestamp(iter->first);
    return true;
  }
  return false;
}

template <typename T>
void THistoryBuffer<T>::clear() {
  buffer_.clear();
}

template <typename T>
bool THistoryBuffer<T>::index_at_t(const Timestamp& t, size_t& idx) const {
  if (!buffer_.empty()) {
    auto it = buffer_.find(t.stamp_ns());
    if (it == buffer_.begin()) {
      idx = 0;
      return true;
    } else if (it != buffer_.end()) {
      idx = std::distance(buffer_.begin(), it);
      return true;
    }
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::exist_at_t(const double t) const {
  return exist_at_t(Timestamp(t));
}

template <typename T>
bool THistoryBuffer<T>::exist_at_t(const Timestamp& t) const {
  if (!buffer_.empty()) {
    auto it = buffer_.find(t.stamp_ns());
    return (it != buffer_.end());
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::exist_after_t(const Timestamp& t) const {
  if (!buffer_.empty()) {
    auto it = upper_bound(t);
    if (it != buffer_.end()) {
      return true;
    }
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::exist_before_t(const Timestamp& t) const {
  if (!buffer_.empty()) {
    auto it = lower_bound(t);
    if (it != buffer_.end()) {
      if (it != buffer_.begin()) {
        return true;
      }
    } else if (buffer_.size()) {
      // corner case: when t is outside the timespan, return last elem.
      return true;
    }
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_oldest_t(Timestamp& t) const {
  if (!buffer_.empty()) {
    t = Timestamp((buffer_.begin())->first);
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_latest_t(Timestamp& t) const {
  if (!buffer_.empty()) {
    t = Timestamp((buffer_.rbegin())->first);
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_oldest(T& elem) const {
  if (!buffer_.empty()) {
    elem = (buffer_.begin())->second;
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_latest(T& elem) const {
  if (!buffer_.empty()) {
    elem = (buffer_.rbegin())->second;
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_at_t(const Timestamp& t, T& elem) const {
  auto it = buffer_.find(t.stamp_ns());
  if (it != buffer_.end()) {
    elem = it->second;
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_at_t(const Timestamp& t, TData& elem) const {
  auto it = buffer_.find(t.stamp_ns());
  if (it != buffer_.end()) {
    elem.data = it->second;
    elem.stamp = Timestamp(it->first);
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_at_t(const double t_sec, T& elem) const {
  return get_at_t(Timestamp(t_sec), elem);
}

template <typename T>
bool THistoryBuffer<T>::get_before_t(const Timestamp& t, T& elem) const {
  auto it = lower_bound(t);
  if (it != buffer_.end()) {
    if (it != buffer_.begin()) {
      auto it_before = --it;
      elem = it_before->second;
      return true;
    }
  } else if (buffer_.size()) {
    auto it_before = --it;
    // corner case: when t is outside the timespan, return last elem.
    elem = it_before->second;
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_before_t(const Timestamp& t, Timestamp& elem) const {
  auto it = lower_bound(t);
  if (it != buffer_.end()) {
    if (it != buffer_.begin()) {
      auto it_before = --it;
      elem = Timestamp(it_before->first);
      return true;
    }
  } else if (buffer_.size()) {
    auto it_before = --it;
    // corner case: when t is outside the timespan, return last elem.
    elem = Timestamp(it_before->first);
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_before_t(const Timestamp& t, TData& elem) const {
  auto it = lower_bound(t);
  if (it != buffer_.end()) {
    if (it != buffer_.begin()) {
      auto it_before = --it;
      elem.data = it_before->second;
      elem.stamp = Timestamp(it_before->first);
      return true;
    }
  } else if (buffer_.size()) {
    auto it_before = --it;
    // corner case: when t is outside the timespan, return last elem.
    elem.data = it_before->second;
    elem.stamp = Timestamp(it_before->first);
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_after_t(const Timestamp& t, T& elem) const {
  auto it = upper_bound(t);
  if (it != buffer_.end()) {
    elem = it->second;
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_after_t(const Timestamp& t, Timestamp& elem) const {
  auto it = upper_bound(t);
  if (it != buffer_.end()) {
    elem = Timestamp(it->first);
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_after_t(const Timestamp& t, TData& elem) const {
  auto it = upper_bound(t);
  if (it != buffer_.end()) {
    elem.data = it->second;
    elem.stamp = Timestamp(it->first);
    return true;
  }
  return false;
}

template <typename T>
bool THistoryBuffer<T>::get_closest_t(const Timestamp& t, T& elem) const {
  TData elem_;
  bool res = get_closest_t(t, elem_);
  if (res) {
    elem = elem_.data;
  }
  return res;
}

template <typename T>
bool THistoryBuffer<T>::get_closest_t(const Timestamp& t, Timestamp& elem) const {
  TData elem_;
  bool res = get_closest_t(t, elem_);
  if (res) {
    elem = elem_.stamp;
  }
  return res;
}

template <typename T>
bool THistoryBuffer<T>::get_closest_t(const Timestamp& t, TData& elem) const {
  if (empty()) {
    return false;
  }
  if (exist_at_t(t)) {
    return get_at_t(t, elem);
  }
  // bounded
  else if (exist_after_t(t) && exist_before_t(t)) {
    Timestamp t_after, t_before;
    if (get_before_t(t, t_before) && get_after_t(t, t_after)) {
      if (t - t_before < t_after - t) {
        return get_before_t(t, elem);
      } else {
        return get_after_t(t, elem);
      }
    }
  }
  // upper bounded
  else if (exist_after_t(t) && !exist_before_t(t)) {
    return get_after_t(t, elem);
  }
  // lower bounded
  else if (!exist_after_t(t) && exist_before_t(t)) {
    return get_before_t(t, elem);
  }
  // no belief exist
  return false;
}

template <typename T>
void THistoryBuffer<T>::remove_at_t(const Timestamp& t) {
  buffer_.erase(t.stamp_ns());
}

template <typename T>
void THistoryBuffer<T>::remove_at_t(const double t) {
  remove_at_t(Timestamp(t));
}

template <typename T>
void THistoryBuffer<T>::remove_before_t(const Timestamp& t) {
  auto it = lower_bound(t);
  if (it != buffer_.end()) {
    buffer_.erase(buffer_.begin(), it);
  }
}

template <typename T>
void THistoryBuffer<T>::remove_before_t(const double t) {
  remove_before_t(Timestamp(t));
}

template <typename T>
void THistoryBuffer<T>::remove_after_t(const Timestamp& t) {
  auto it = upper_bound(t);
  if (it != buffer_.end()) {
    buffer_.erase(it, buffer_.end());
  }
}

template <typename T>
void THistoryBuffer<T>::remove_after_t(const double t) {
  remove_after_t(Timestamp(t));
}

template <typename T>
void THistoryBuffer<T>::remove_every_N(size_t const subsample) {
  size_t cnt = 0;
  for (auto it = buffer_.cbegin(); it != buffer_.cend() /* not hoisted */; /* no increment */) {
    if (cnt % subsample == 0) {
      it = buffer_.erase(it);  // or "it = m.erase(it)" since C++11
    } else {
      ++it;
    }
    cnt++;
  }
}

template <typename T>
void THistoryBuffer<T>::subsample_by_N(size_t const subsample) {
  size_t cnt = 0;
  for (auto it = buffer_.cbegin(); it != buffer_.cend() /* not hoisted */; /* no increment */) {
    if (cnt % subsample != 0) {
      it = buffer_.erase(it);  // or "it = m.erase(it)" since C++11
    } else {
      ++it;
    }
    cnt++;
  }
}
}  // namespace ikf

#endif  // IKF_THISTORYBUFFER_HPP
