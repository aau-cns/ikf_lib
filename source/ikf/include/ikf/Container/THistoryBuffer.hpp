/******************************************************************************
* FILENAME:     THistoryBuffer.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       Roland Jung
* MAIL:         jungr-ait@github
* VERSION:      v0.0.0
* CREATION:     19.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_THISTORYBUFFER_HPP
#define IKF_THISTORYBUFFER_HPP
#include <ikf/ikf_api.h>
#include <ikf/Container/TStampedData.hpp>
#include <list>
#include <numeric>
#include <ostream>
#include <set>
#include <map>
#include <algorithm>
#include <functional>

namespace ikf
{
  // TODO: check if std::map is faster than set!

  /**
   * @brief The THistoryBuffer class to store data with associated timestamps
   */
  template<typename T>
  class IKF_API THistoryBuffer
  {
    public:


      typedef TStampedData<T> TData;
      typedef std::map<std::int64_t, T> TContainer;
      void insert(T const& data, Timestamp const t);
      void insert(T const& data, double const t_sec);
      void insert(TData const& data);
      size_t size() const;
      void clear();



      void set(TContainer const& buffer){
        buffer_ = buffer;
      }

      bool exist_at_t(double const t) const;
      bool exist_at_t(Timestamp const&t) const;
      bool exist_after_t(Timestamp const&t) const;

      bool get_oldest_t(Timestamp &t) const;
      bool get_latest_t(Timestamp &t) const;
      bool get_oldest(T & elem) const;
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
      template<typename BinaryOperation > // Ret fun(const Type1 &a, const Type2 &b)
      inline T accumulate_between_t1_t2(Timestamp const& t1, Timestamp const& t2, T init, BinaryOperation op ) const {

        if (t1 < t2 && size() > 0) {
          auto __first = lower_bound(t1);
          auto __last = upper_bound(t2);
          for(; __first != __last; ++__first)
            init = op(init, __first->second);
          return init;
        }
        return init;
      }

      template<typename BinaryOperation > // Ret fun(const Type1 &a, const Type2 &b)
      inline T accumulate(T init, BinaryOperation op ) {
        auto __first = buffer_.begin();
        auto __last =  buffer_.end();
        for(; __first != __last; ++__first)
          init = op(init, __first->second);
        return init;
      }

      template<typename Operation >
      inline void foreach_between_t1_t2(Timestamp const& t1, Timestamp const& t2, Operation op ) {
        if (t1 < t2 && size() > 0) {
          auto __first = lower_bound(t1);
          auto __last = upper_bound(t2);

          for(; __first != __last; ++__first)
            op(__first->second);
        }
      }

      template<typename Operation >
      inline void foreach(Operation op ) {
        auto __first = buffer_.begin();
        auto __last =  buffer_.end();
        for(; __first != __last; ++__first)
            op(__first->second);

      }

      bool get_at_t(Timestamp const& t, T& elem) const;
      bool get_at_t(double const t_sec, T& elem) const;
      bool get_before_t(Timestamp const& t, T& elem) const;
      bool get_before_t( Timestamp const& t, Timestamp& elem) const;
      bool get_before_t(Timestamp const& t, TData &elem) const;
      bool get_after_t(Timestamp const& t, T& elem) const;
      bool get_after_t(Timestamp const& t, Timestamp& elem) const;
      bool get_after_t(Timestamp const& t, TData& elem) const;
      void remove_at_t(Timestamp const& t);
      void remove_at_t(double const t);
      void remove_before_t(Timestamp const& t);
      void remove_before_t(double const t);
      void remove_after_t(Timestamp const& t);
      void remove_after_t(double const t);

      friend std::ostream& operator<< (std::ostream& out, const THistoryBuffer<T>& obj)  {
          out << "THistoryBuffer: len=" << obj.size() << std::endl;
          for (auto const& elem : obj.buffer_)
          {
              out << "* t=" << elem.first << ", data=" << elem.second << "\n";
          }
          return out;
      }
    protected:
      friend std::ostream;
      TContainer buffer_;
    private:
       void insert_sorted(Timestamp const& t, T const& elem){
          buffer_[t.stamp_ns()] = elem;
       }

       // https://stackoverflow.com/a/72835502
       typename TContainer::const_iterator upper_bound(Timestamp const& t) const {
          return std::upper_bound(buffer_.begin(), buffer_.end(), t.stamp_ns(), [](const std::int64_t &value, const std::pair<std::int64_t, T> iter) {
            return value < iter.first;
          });
       }

       typename TContainer::const_iterator lower_bound(Timestamp const& t) const {
          return std::lower_bound(buffer_.begin(), buffer_.end(), t.stamp_ns(), [](const std::pair<std::int64_t, T> &iter, const std::int64_t &value) {
            return iter.first < value;
          });
       }

  };

  template<typename T>
  void THistoryBuffer<T>::insert(const T &data, const Timestamp t) {
      insert_sorted(t, data);
  }

  template<typename T>
  void THistoryBuffer<T>::insert(const T &data, const double t_sec){
      insert_sorted(Timestamp(t_sec), data);
  }

  template<typename T>
  void THistoryBuffer<T>::insert(const TData &data){
      insert_sorted(data.stamp, data.data);
  }

  template<typename T>
  size_t THistoryBuffer<T>::size() const {
      return buffer_.size();
  }

  template<typename T>
  void THistoryBuffer<T>::clear() {
      buffer_.clear();
  }

  template<typename T>
  bool THistoryBuffer<T>::exist_at_t(const double t) const {
      return exist_at_t(Timestamp(t));
  }

  template<typename T>
  bool THistoryBuffer<T>::exist_at_t(const Timestamp &t) const {
      auto it = buffer_.find(t.stamp_ns());
      return (it != buffer_.end());
  }

  template<typename T>
  bool THistoryBuffer<T>::exist_after_t(const Timestamp &t) const {
      auto it = upper_bound(t);
      if (it != buffer_.end()) {
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_oldest_t(Timestamp &t) const{
      if (!buffer_.empty()) {
        t = Timestamp((buffer_.begin())->first);
        return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_latest_t(Timestamp &t) const{
      if (!buffer_.empty()) {
        t = Timestamp((buffer_.rbegin())->first);
        return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_oldest(T &elem) const {
      if (!buffer_.empty()) {
          elem = (buffer_.begin())->second;
          return true;
      }
      return false;
  }


  template<typename T>
  bool THistoryBuffer<T>::get_latest(T &elem) const{
      if (!buffer_.empty()) {
          elem = (buffer_.rbegin())->second;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_at_t(const Timestamp &t, T &elem) const {
      auto it = buffer_.find(t.stamp_ns());
      if (it != buffer_.end()) {
          elem = it->second;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_at_t(const double t_sec, T &elem) const {
      return get_at_t(Timestamp(t_sec), elem);
  }

  template<typename T>
  bool THistoryBuffer<T>::get_before_t(const Timestamp &t, T &elem) const {
      auto it = lower_bound(t);
      if (it != buffer_.end()) {
          if (it != buffer_.begin()){
              auto it_before = --it;
              elem = it_before->second;
              return true;
          }
      }
      else if(buffer_.size()){
          auto it_before = --it;
          // corner case: when t is outside the timespan, return last elem.
          elem = it_before->second;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_before_t(const Timestamp &t, Timestamp &elem) const {
      auto it = lower_bound(t);
      if (it != buffer_.end()) {
          if (it != buffer_.begin()){
              auto it_before = --it;
              elem = it_before->first;
              return true;
          }
      }
      else if(buffer_.size()){
          auto it_before = --it;
        // corner case: when t is outside the timespan, return last elem.
        elem = it_before->first;
        return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_before_t(const Timestamp &t, TData &elem) const {
      auto it = lower_bound(t);
      if (it != buffer_.end()) {
        if (it != buffer_.begin()){
              auto it_before = --it;
              elem.data = it_before->second;
              elem.stamp = Timestamp(it_before->first);
              return true;
        }
      }
      else if(buffer_.size()){
        auto it_before = --it;
        // corner case: when t is outside the timespan, return last elem.
        elem.data = it_before->second;
        elem.stamp = Timestamp(it_before->first);
        return true;
      }
      return false;
  }


  template<typename T>
  bool THistoryBuffer<T>::get_after_t(const Timestamp &t, T &elem) const{
      auto it = upper_bound(t);
      if (it != buffer_.end()) {
          elem = it->second;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_after_t(const Timestamp &t, Timestamp &elem) const {
      auto it = upper_bound(t);
      if (it != buffer_.end()) {
          elem = it->first;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_after_t(const Timestamp &t, TData &elem) const {
      auto it = upper_bound(t);
      if (it != buffer_.end()) {
          elem.data = it->second;
          elem.stamp = Timestamp(it->first);
          return true;
      }
      return false;
  }

  template<typename T>
  void THistoryBuffer<T>::remove_at_t(const Timestamp &t) {
      buffer_.erase(t.stamp_ns());
  }

  template<typename T>
  void THistoryBuffer<T>::remove_at_t(const double t) {
      remove_at_t(Timestamp(t));
  }

  template<typename T>
  void THistoryBuffer<T>::remove_before_t(const Timestamp &t) {
      auto it = lower_bound(t);
      if (it != buffer_.end()) {
          buffer_.erase(buffer_.begin(), it);
      }
  }

  template<typename T>
  void THistoryBuffer<T>::remove_before_t(const double t) {
      remove_before_t(Timestamp(t));
  }

  template<typename T>
  void THistoryBuffer<T>::remove_after_t(const Timestamp &t) {
      auto it = upper_bound(t);
      if (it != buffer_.end()) {
          buffer_.erase(it, buffer_.end());
      }
  }

  template<typename T>
  void THistoryBuffer<T>::remove_after_t(const double t) {
      remove_after_t(Timestamp(t));
  }


  } // namespace ikf

#endif // IKF_THISTORYBUFFER_HPP
