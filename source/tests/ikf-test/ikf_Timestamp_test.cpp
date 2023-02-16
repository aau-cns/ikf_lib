/******************************************************************************
* FILENAME:     ikf_Timestamp_test.cpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
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
#include <gmock/gmock.h>
#include <ikf/Container/Timestamp.hpp>

class IKF_Timestamp_test : public testing::Test
{
public:
};

TEST_F(IKF_Timestamp_test, ctor_init)
{
  {
    ikf::Timestamp t(0.0);
    EXPECT_TRUE(t.nsec == 0);
    EXPECT_TRUE(t.sec == 0);
  }
  {
    ikf::Timestamp t(1.0);
    EXPECT_TRUE(t.nsec == 0);
    EXPECT_TRUE(t.sec == 1);
  }

  {
    ikf::Timestamp t(1.000000001);
    EXPECT_TRUE(t.nsec == 1);
    EXPECT_TRUE(t.sec == 1);
  }
  {
    ikf::Timestamp t(1.0000000001);
    EXPECT_TRUE(t.nsec == 0);
    EXPECT_TRUE(t.sec == 1);
  }
  {
    ikf::Timestamp t(1.000000000999);
    EXPECT_TRUE(t.nsec == 1);
    EXPECT_TRUE(t.sec == 1);
  }
  {
    ikf::Timestamp t(1,1);
    EXPECT_TRUE(t.nsec == 1);
    EXPECT_TRUE(t.sec == 1);
  }
  {
    ikf::Timestamp t(1.000000001);
    std::int64_t stamp_ns = t.stamp_ns();
    std::cout << "stamp_ns = " <<  t.stamp_ns() << std::endl;
    EXPECT_TRUE(stamp_ns == 1000000001);
  }
  {
    std::int64_t stamp_ns = 1000000001;
    ikf::Timestamp t(stamp_ns);
    EXPECT_TRUE(t.nsec == 1);
    EXPECT_TRUE(t.sec == 1);
  }

}
