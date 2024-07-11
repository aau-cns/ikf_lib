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
    ikf::Timestamp t(0.000000001);
    EXPECT_TRUE(t.nsec == 1);
    EXPECT_TRUE(t.sec == 0);
  }
  {
    ikf::Timestamp t(1.0000000001);
    EXPECT_EQ(t.nsec, 0);
    EXPECT_EQ(t.sec, 1);
  }
  {
    ikf::Timestamp t(1.000000000999);
    EXPECT_EQ(t.nsec, 1);
    EXPECT_EQ(t.sec, 1);
  }
  {
    ikf::Timestamp t(1,1);
    EXPECT_EQ(t.nsec, 1);
    EXPECT_EQ(t.sec, 1);
  }
  {
    ikf::Timestamp t(1.999999999);
    EXPECT_EQ(t.nsec, 999999999);
    EXPECT_EQ(t.sec, 1);
  }
  {
    ikf::Timestamp t(8321499136.0);
    EXPECT_EQ(t.nsec, 0);
    EXPECT_EQ(t.sec, __INT64_C(0x1F0000000));
  }
}
TEST_F(IKF_Timestamp_test, from_pos) {
  {
    ikf::Timestamp t(1.000000001);
    std::int64_t stamp_ns = t.stamp_ns();
    std::cout << "stamp_ns = " <<  t.stamp_ns() << std::endl;
    EXPECT_EQ(stamp_ns, 1000000001);
  }
  {
    std::int64_t stamp_ns = 1000000001;
    ikf::Timestamp t(stamp_ns);
    EXPECT_EQ(t.nsec, 1);
    EXPECT_EQ(t.sec, 1);
  }
  {
    ikf::Timestamp t(1.000001);
    std::int64_t stamp_us = t.stamp_us();
    std::cout << "stamp_us = " << t.stamp_us() << std::endl;
    EXPECT_EQ(stamp_us, 1000001);
  }
  {
    std::int64_t stamp_us = 1000001;
    ikf::Timestamp t;
    t.from_stamp_us(stamp_us);
    EXPECT_EQ(t.nsec, 1000);
    EXPECT_EQ(t.sec, 1);
  }
  {
    ikf::Timestamp t(1.001);
    std::int64_t stamp_ms = t.stamp_ms();
    std::cout << "stamp_ms = " << t.stamp_ms() << std::endl;
    EXPECT_EQ(stamp_ms, 1001);
  }
  {
    std::int64_t stamp_ms = 1001;
    ikf::Timestamp t;
    t.from_stamp_ms(stamp_ms);
    EXPECT_EQ(t.nsec, 1000000);
    EXPECT_EQ(t.sec, 1);
  }
}

TEST_F(IKF_Timestamp_test, from_neg) {
  {
    ikf::Timestamp t(-1.000000001);
    std::int64_t stamp_ns = t.stamp_ns();
    std::cout << "stamp_ns = " << t.stamp_ns() << std::endl;
    EXPECT_EQ(stamp_ns, -1000000001);
  }
  {
    std::int64_t stamp_ns = -1000000001;
    ikf::Timestamp t(stamp_ns);
    EXPECT_EQ(t.nsec, -1);
    EXPECT_EQ(t.sec, -1);
  }
  {
    ikf::Timestamp t(-1.000001);
    std::int64_t stamp_us = t.stamp_us();
    std::cout << "stamp_us = " << t.stamp_us() << std::endl;
    EXPECT_EQ(stamp_us, -1000001);
  }
  {
    std::int64_t stamp_us = -1000001;
    ikf::Timestamp t;
    t.from_stamp_us(stamp_us);
    EXPECT_EQ(t.nsec, -1000);
    EXPECT_EQ(t.sec, -1);
  }
  {
    ikf::Timestamp t(-1.001);
    std::int64_t stamp_ms = t.stamp_ms();
    std::cout << "stamp_ms = " << t.stamp_ms() << std::endl;
    EXPECT_EQ(stamp_ms, -1001);
  }
  {
    std::int64_t stamp_ms = -1001;
    ikf::Timestamp t;
    t.from_stamp_ms(stamp_ms);
    EXPECT_EQ(t.nsec, -1000000);
    EXPECT_EQ(t.sec, -1);
  }
}

TEST_F(IKF_Timestamp_test, from_neg_below_zero) {
  {
    ikf::Timestamp t(-0.000000001);
    std::int64_t stamp_ns = t.stamp_ns();
    std::cout << "stamp_ns = " << t.stamp_ns() << std::endl;
    EXPECT_EQ(stamp_ns, -1);
  }
  {
    std::int64_t stamp_ns = -0000000001;
    ikf::Timestamp t(stamp_ns);
    EXPECT_EQ(t.nsec, -1);
    EXPECT_EQ(t.sec, 0);
  }
  {
    ikf::Timestamp t(-0.000001);
    std::int64_t stamp_us = t.stamp_us();
    std::cout << "stamp_us = " << t.stamp_us() << std::endl;
    EXPECT_EQ(stamp_us, -0000001);
  }
  {
    std::int64_t stamp_us = -1;
    ikf::Timestamp t;
    t.from_stamp_us(stamp_us);
    EXPECT_EQ(t.nsec, -1000);
    EXPECT_EQ(t.sec, 0);
  }
  {
    ikf::Timestamp t(-0.001);
    std::int64_t stamp_ms = t.stamp_ms();
    std::cout << "stamp_ms = " << t.stamp_ms() << std::endl;
    EXPECT_EQ(stamp_ms, -1);
  }
  {
    std::int64_t stamp_ms = -1;
    ikf::Timestamp t;
    t.from_stamp_ms(stamp_ms);
    EXPECT_EQ(t.nsec, -1000000);
    EXPECT_EQ(t.sec, 0);
  }
}
TEST_F(IKF_Timestamp_test, add) {
  {
    ikf::Timestamp t1(1.000000001);
    ikf::Timestamp t2(1.000000001);

    auto t = t1 + t2;
    std::int64_t stamp_ns = t.stamp_ns();
    EXPECT_EQ(stamp_ns, 2000000002);
  }
  {
    ikf::Timestamp t1(1.000000001);
    ikf::Timestamp t2(-1.000000001);

    auto t = t1 + t2;
    std::int64_t stamp_ns = t.stamp_ns();
    EXPECT_EQ(stamp_ns, 0);
  }

  {
    ikf::Timestamp t1(1.000000001);
    ikf::Timestamp t2(-0.000000001);

    auto t = t1 + t2;
    std::int64_t stamp_ns = t.stamp_ns();
    EXPECT_EQ(stamp_ns, 1000000000);
  }
  {
    ikf::Timestamp t1(-1.000000001);
    ikf::Timestamp t2(0.000000001);

    auto t = t1 + t2;
    std::int64_t stamp_ns = t.stamp_ns();
    EXPECT_EQ(stamp_ns, -1000000000);
  }
}
TEST_F(IKF_Timestamp_test, sub) {
  {
    ikf::Timestamp t1(-1.000000001);
    ikf::Timestamp t2(1.000000001);

    auto t = t1 - t2;
    std::int64_t stamp_ns = t.stamp_ns();
    EXPECT_EQ(stamp_ns, -2000000002);
  }
  {
    ikf::Timestamp t1(1.000000001);
    ikf::Timestamp t2(1.000000001);

    auto t = t1 - t2;
    std::int64_t stamp_ns = t.stamp_ns();
    EXPECT_EQ(stamp_ns, 0);
  }

  {
    ikf::Timestamp t1(1.000000001);
    ikf::Timestamp t2(0.000000001);

    auto t = t1 - t2;
    std::int64_t stamp_ns = t.stamp_ns();
    EXPECT_EQ(stamp_ns, 1000000000);
  }
  {
    ikf::Timestamp t1(1.000000001);
    ikf::Timestamp t2(-0.000000001);

    auto t = t1 - t2;
    std::int64_t stamp_ns = t.stamp_ns();
    EXPECT_EQ(stamp_ns, 1000000002);
  }
}
