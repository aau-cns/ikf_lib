#ifndef IKF_TBUFFER_HPP
#define IKF_TBUFFER_HPP

#include <ikf/ikf_api.h>
#include <ikf/container/Timestamp.hpp>
#include <deque>
#include <type_traits>
#include <mutex>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>

namespace ikf
{
  /**
   * @brief Buffer class to be used only with typed derived from Timestamped
   *
   * @tparam T type to be used for the buffer
   * @tparam max_buffer_size=256
   */
  template <class T, size_t max_buffer_size=256>
  class IKF_API TBuffer
  {
      static_assert(std::is_base_of<Timestamped, T>::value, "Type T must inherit from Class Timestamped");
    public:

      /**
       * @brief Construct a new TBuffer object
       */
      TBuffer();

      /**
       * @brief Set the max buffer size
       * @param size
       */
      void set_max_buffer_size(const unsigned size);

      /**
       * @brief Get the max buffer size
       * @return unsigned
       */
      unsigned get_max_buffer_size() const;

      /**
       * @brief Get the buffer size
       * @return unsigned
       */
      unsigned get_size() const;

      /**
       * @brief is_newer
       * @param data
       * @return
       */
      bool is_newer(T const& data);

      /**
       * @brief Insert data at the right position in the buffer based on the timestamp
       *
       * @param data
       */
      void insert_sorted(const T& data);

      /**
       * @brief return first buffer element (oldest)
       *
       * @return T
       */
      bool get_oldest(T& elem) const;

      /**
       * @brief return last buffer element (newest)
       *
       * @return T
       */
      bool get_latest(T& elem) const;

      /**
       * @brief Get the buffer entry at time t
       *
       * @param comp_t timestamp to be searched for
       * @param data returns the corresponding buffer entry
       * @return true successfull
       * @return false no buffer entry
       */
      bool get_buffer_at_t(const Timestamp &comp_t, T& data) const;

      /**
       * @brief Get the buffer entry relative to entry at t
       *
       * @param comp_t comparing timestamp
       * @param rel relative position to timestamp
       * @param data returns corresponding buffer entry
       * @return true
       * @return false
       */
      bool get_buffer_at_t_relative(const Timestamp &comp_t, const int &rel, T& data) const;
    private:

      mutable std::mutex mtx_;
      unsigned max_buffer_size_;
      std::deque<T> buffer_;

      /**
       * @brief Comparing timestamps
       */
      static bool time_comp_small_to_greater(const Timestamped &a, const Timestamped &b);

      /**
       * @brief Search buffer for entry with certain timestamp
       *
       * @param search_time
       * @return buffer "index" if found an "-1" if no buffer entry was found
       */
      signed search_t_in_buffer(const Timestamp &search_time) const;
  };

  template<class T, size_t max_buffer_size>
  TBuffer<T, max_buffer_size>::TBuffer() : max_buffer_size_(max_buffer_size) {}

  template<class T, size_t max_buffer_size>
  void TBuffer<T, max_buffer_size>::set_max_buffer_size(const unsigned size)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    max_buffer_size_ = size;
  }

  template<class T, size_t max_buffer_size>
  unsigned TBuffer<T, max_buffer_size>::get_max_buffer_size() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return max_buffer_size_;
  }

  template<class T, size_t max_buffer_size>
  unsigned TBuffer<T, max_buffer_size>::get_size() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return buffer_.size();
  }

  template<class T, size_t max_buffer_size>
  bool TBuffer<T, max_buffer_size>::is_newer(const T &data)
  {
    T elem;
    if(!this->get_latest(elem))
    {
      return true;
    }
    return time_comp_small_to_greater(elem, data);
  }

  template<class T, size_t max_buffer_size>
  void TBuffer<T, max_buffer_size>::insert_sorted(const T &data)
  {
    bool newer = is_newer(data);

    std::lock_guard<std::mutex> lock(mtx_);

    buffer_.push_back(data);
    if(!newer)
    {
      std::sort(buffer_.begin(), buffer_.end(), time_comp_small_to_greater);
    }

    if (buffer_.size() >= max_buffer_size_)
    {
      buffer_.pop_front();
    }
  }

  template<class T, size_t max_buffer_size>
  bool TBuffer<T, max_buffer_size>::get_oldest(T &elem) const
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if( buffer_.size() == 0 )
    {
      return false;
    }

    elem = buffer_.front();
    return true;
  }

  template<class T, size_t max_buffer_size>
  bool TBuffer<T, max_buffer_size>::get_latest(T &elem) const
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if( buffer_.size() == 0 )
    {
      return false;
    }

    elem = buffer_.back();
    return true;
  }

  template<class T, size_t max_buffer_size>
  bool TBuffer<T, max_buffer_size>::get_buffer_at_t(const Timestamp &comp_t, T &data) const
  {
    std::lock_guard<std::mutex> lock(mtx_);

    const int buffer_index = search_t_in_buffer(comp_t);

    if(buffer_index < 0)
    {
      return false;
    }

    data = buffer_[buffer_index];
    return true;
  }

  template<class T, size_t max_buffer_size>
  bool TBuffer<T, max_buffer_size>::get_buffer_at_t_relative(const Timestamp &comp_t, const int &rel, T &data) const
  {
    std::lock_guard<std::mutex> lock(mtx_);

    const int buffer_index = search_t_in_buffer(comp_t);

    if(buffer_index < 0)
    {
      return false;
    }

    const int relative_element = buffer_index + rel;

    if(relative_element < 0)
    {
      return false;
    }

    if(unsigned(relative_element) > buffer_.size()-1)
    {
      return false;
    }

    data = buffer_[relative_element];
    return true;
  }

  template<class T, size_t max_buffer_size>
  bool TBuffer<T, max_buffer_size>::time_comp_small_to_greater(const Timestamped &a, const Timestamped &b)
  {
    return a < b;
  }

  template<class T, size_t max_buffer_size>
  signed TBuffer<T, max_buffer_size>::search_t_in_buffer(const Timestamp &search_time) const
  {
    const unsigned current_buffer_size = buffer_.size();

    if(current_buffer_size == 0)
    {
      return -1;
    }

    const unsigned last_buffer_index = current_buffer_size - 1;

    // ToDo: This will also catch index 0.
    // address this in the test
    const double comp_t_sec = search_time.to_sec();

    for (unsigned i = current_buffer_size; i-- > 0; )
    {
      const double buffer_t_sec = buffer_[i].to_sec();

      if (buffer_t_sec <= comp_t_sec)
      {
        if (buffer_t_sec == comp_t_sec)
        {
          return i;
        }

        // ensure i+1 does exist
        if( (i+1) > last_buffer_index )
        {
          // i+1 does not exist, return most current
          return i;
        }

        if(std::abs(buffer_[i].to_sec() - comp_t_sec) < std::abs(buffer_[i+1].to_sec() - comp_t_sec))
        {
          return i;
        }
        else
        {
          // this will also be returned if distance is equal
          return i+1;
        }

      }
    }

    std::cout << "Could not find buffer element that is smaller in time than given timestamp." << std::endl;
    return -1; // no element found
  }

}

#endif // IKF_TBUFFER_HPP
