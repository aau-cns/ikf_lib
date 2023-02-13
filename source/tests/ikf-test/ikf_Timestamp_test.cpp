/******************************************************************************
* FILENAME:     ikf_Timestamp_test.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
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
