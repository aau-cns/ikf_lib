/******************************************************************************
* FILENAME:     TStampedData.hpp
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
#ifndef IKF_TSTAMPEDDATA_HPP
#define IKF_TSTAMPEDDATA_HPP
#include <ikf/ikf_api.h>
#include <ikf/Container/Timestamp.hpp>
#include <cstdint>

namespace ikf
{
    template<typename T>
    struct IKF_API TStampedData
    {
        TStampedData() {}
        TStampedData(T const& data_, Timestamp const& stamp_):data(data_), stamp(stamp_) {}
        TStampedData(T const& data_, double const t_sec_):data(data_), stamp(Timestamp(t_sec_)) {}
        TStampedData(T const& data_, std::int64_t const t_ns_):data(data_), stamp(Timestamp(t_ns_)) {}
        T data;
        Timestamp stamp;

        bool operator < (TStampedData const& rhs)  const
        {
            if(stamp.sec < rhs.stamp.sec)
            {
                return true;
            }
            else if(stamp.sec == rhs.stamp.sec)
            {
                if(stamp.nsec < rhs.stamp.nsec)
                {
                    return true;
                }
            }
            return false;
        }

        bool operator < (Timestamp const& rhs)  const
        {
            if(stamp.sec < rhs.sec)
            {
                return true;
            }
            else if(stamp.sec == rhs.sec)
            {
                if(stamp.nsec < rhs.nsec)
                {
                    return true;
                }
            }
            return false;
        }

        bool operator < (double const& rhs)  const
        {
            return (stamp.to_sec() < rhs);
        }


        bool operator==(TStampedData const& rhs) const  {
            return (stamp.sec == rhs.stamp.sec) && (stamp.nsec == rhs.stamp.nsec);
        }

        bool operator==(Timestamp const& rhs) const  {
            return (stamp.sec == rhs.sec) && (stamp.nsec == rhs.nsec);
        }
        bool operator==(double const& rhs) const  {
            return (stamp.to_sec() == rhs);
        }

        bool operator > (TStampedData const& rhs) const  {
            return !(*this == rhs ) && !(*this < rhs);
        }

        bool operator > (Timestamp const& rhs) const  {
            return !(*this == rhs ) && !(*this < rhs);
        }

        bool operator > (double const& rhs) const  {
            return !(*this == rhs ) && !(*this < rhs);
        }
    };

    template<typename T>
    bool operator<(double const lhs, TStampedData<T> const& rhs)  {return Timestamp(lhs) < rhs.stamp;}

    template<typename T>
    bool operator<(Timestamp const& lhs, TStampedData<T> const& rhs)  {return lhs < rhs.stamp;}

    //template<typename T>
    //bool operator<(Timestamp const& lhs, Timestamp const& rhs)  {return lhs < rhs; }
    //template<typename T>
    //bool operator<(Timestamp const& lhs, const TStampedData<T>& rhs)  {return lhs < rhs.stamp;}

} // namespace ikf
#endif // IKF_TSTAMPEDDATA_HPP
