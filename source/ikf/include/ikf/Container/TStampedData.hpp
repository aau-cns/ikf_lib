/******************************************************************************
* FILENAME:     TStampedData.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       Roland Jung
* MAIL:         jungr-ait@github
* VERSION:      v0.0.0
* CREATION:     19.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
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
