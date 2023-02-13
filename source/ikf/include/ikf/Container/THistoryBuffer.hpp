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
      typedef std::set<TStampedData<T>> TContainer;
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
      bool get_oldest(TData& elem) const;
      bool get_oldest(T & elem) const;
      bool get_latest(TData& elem) const;
      bool get_latest(T& elem) const;

      THistoryBuffer get_between_t1_t2(Timestamp const& t1, Timestamp const& t2) {
        if (t1 < t2 && size() > 0) {
          auto itlow = std::lower_bound(buffer_.begin(), buffer_.end(), t1);
          auto itup = std::upper_bound(buffer_.begin(), buffer_.end(), t2);
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
          auto __first = std::lower_bound(buffer_.begin(), buffer_.end(), t1);
          auto __last = std::upper_bound(buffer_.begin(), buffer_.end(), t2);

          for(; __first != __last; ++__first)
            init = op(init, __first->data);
          return init;
        }
        return init;
      }

      template<typename BinaryOperation > // Ret fun(const Type1 &a, const Type2 &b)
      inline T accumulate(T init, BinaryOperation op ) {
        auto __first = buffer_.begin();
        auto __last =  buffer_.end();
        for(; __first != __last; ++__first)
          init = op(init, __first->data);
        return init;
      }

      template<typename Operation >
      inline void foreach_between_t1_t2(Timestamp const& t1, Timestamp const& t2, Operation op ) {
        if (t1 < t2 && size() > 0) {
          auto __first = std::lower_bound(buffer_.begin(), buffer_.end(), t1);
          auto __last = std::upper_bound(buffer_.begin(), buffer_.end(), t2);

          for(; __first != __last; ++__first)
            op(__first->data);
        }
      }

      template<typename Operation >
      inline void foreach(Operation op ) {
        auto __first = buffer_.begin();
        auto __last =  buffer_.end();
        for(; __first != __last; ++__first)
            op(__first->data);

      }

      bool get_at_t(Timestamp const& t, T& elem) const;
      bool get_at_t(Timestamp const& t, TData& elem) const;
      bool get_at_t(double const t_sec, T& elem) const;
      bool get_before_t(Timestamp const& t, T& elem) const;
      bool get_before_t( Timestamp const& t, Timestamp& elem) const;
      bool get_before_t( Timestamp const& t, TData& elem) const;
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
              out << "* t=" << elem.stamp << ", data=" << elem.data << "\n";
          }
          return out;
      }
    protected:
      friend std::ostream;
      TContainer buffer_;
    private:
      typename TContainer::iterator insert_sorted(TData const& elem){
        typename TContainer::iterator it_found = buffer_.find(elem);
        if (it_found != buffer_.end()){
           buffer_.erase(it_found);  // replacing in to possible in std::set
        }
        typename TContainer::iterator it = std::upper_bound( buffer_.begin(), buffer_.end(), elem );
        return buffer_.insert(it, elem);
       }

  };

  template<typename T>
  void THistoryBuffer<T>::insert(const T &data, const Timestamp t) {
      insert(TData(data, t));
  }

  template<typename T>
  void THistoryBuffer<T>::insert(const T &data, const double t_sec){
      insert(TData(data, t_sec));
  }

  template<typename T>
  void THistoryBuffer<T>::insert(const TData &data){
      insert_sorted(data);
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
      auto it = std::find( buffer_.begin(), buffer_.end(), t );
      return (it != buffer_.end());
  }

  template<typename T>
  bool THistoryBuffer<T>::exist_after_t(const Timestamp &t) const {
      auto it = std::upper_bound(buffer_.begin(), buffer_.end(), t);
      if (it != buffer_.end()) {
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_oldest_t(Timestamp &t) const{
      if (!buffer_.empty()) {
        t = (buffer_.begin())->stamp;
        return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_latest_t(Timestamp &t) const{
      if (!buffer_.empty()) {
        t = (buffer_.rbegin())->stamp;
        return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_oldest(TData &elem) const{
      if (!buffer_.empty()) {
        elem = (*buffer_.begin());
        return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_oldest(T &elem) const {
      if (!buffer_.empty()) {
          elem = (buffer_.begin())->data;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_latest(TData &elem) const{
      if (!buffer_.empty()) {
          elem = (*buffer_.rbegin());
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_latest(T &elem) const{
      if (!buffer_.empty()) {
          elem = (buffer_.rbegin())->data;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_at_t(const Timestamp &t, T &elem) const {
      auto it = std::find( buffer_.begin(), buffer_.end(), t );
      if (it != buffer_.end()) {
          elem = it->data;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_at_t(const Timestamp &t, TData &elem) const {
      auto it = std::find( buffer_.begin(), buffer_.end(), t );
      if (it != buffer_.end()) {
          elem = *it;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_at_t(const double t_sec, T &elem) const {
      auto it = std::find( buffer_.begin(), buffer_.end(), t_sec );
      if (it != buffer_.end()) {
          elem = it->data;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_before_t(const Timestamp &t, T &elem) const {
      auto it = std::lower_bound(buffer_.begin(), buffer_.end(), t);
      if (it != buffer_.end()) {
          if (it != buffer_.begin()){
              elem = (--it)->data;
              return true;
          }
      }
      else if(buffer_.size()){
          // corner case: when t is outside the timespan, return last elem.
          elem = (--it)->data;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_before_t(const Timestamp &t, Timestamp &elem) const {
      auto it = std::lower_bound(buffer_.begin(), buffer_.end(), t);
      if (it != buffer_.end()) {
          if (it != buffer_.begin()){
              elem = (--it)->stamp;
              return true;
          }
      }
      else if(buffer_.size()){
        // corner case: when t is outside the timespan, return last elem.
        elem = (--it)->stamp;
        return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_before_t(const Timestamp &t, TData &elem) const {
      auto it = std::lower_bound(buffer_.begin(), buffer_.end(), t);
      if (it != buffer_.end()) {
          if (it != buffer_.begin()){
              elem = *(--it);
              return true;
          }
      }
      else if(buffer_.size()){
          // corner case: when t is outside the timespan, return last elem.
          elem = *(--it);
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_after_t(const Timestamp &t, T &elem) const{
      auto it = std::upper_bound(buffer_.begin(), buffer_.end(), t);
      if (it != buffer_.end()) {
          elem = it->data;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_after_t(const Timestamp &t, Timestamp &elem) const {
      auto it = std::upper_bound(buffer_.begin(), buffer_.end(), t);
      if (it != buffer_.end()) {
          elem = it->stamp;
          return true;
      }
      return false;
  }

  template<typename T>
  bool THistoryBuffer<T>::get_after_t(const Timestamp &t, TData &elem) const{
      auto it = std::upper_bound(buffer_.begin(), buffer_.end(), t);
      if (it != buffer_.end()) {
          elem = *it;
          return true;
      }
      return false;
  }

  template<typename T>
  void THistoryBuffer<T>::remove_at_t(const Timestamp &t) {
      auto it = std::find( buffer_.begin(), buffer_.end(), t );
      if (it != buffer_.end()) {
          buffer_.erase(it);
      }
  }

  template<typename T>
  void THistoryBuffer<T>::remove_at_t(const double t) {
      auto it = std::find( buffer_.begin(), buffer_.end(), t);
      if (it != buffer_.end()) {
          buffer_.erase(it);
      }
  }

  template<typename T>
  void THistoryBuffer<T>::remove_before_t(const Timestamp &t) {
      auto it = std::lower_bound(buffer_.begin(), buffer_.end(), t);
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
      auto it = std::upper_bound(buffer_.begin(), buffer_.end(), t);
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
