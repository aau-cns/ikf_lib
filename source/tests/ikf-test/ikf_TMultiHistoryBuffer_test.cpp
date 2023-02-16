/******************************************************************************
* FILENAME:     ikf_TMultiHistoryBuffer_test.cpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     15.02.2023
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
#include <ikf/Container/TMultiHistoryBuffer.hpp>
#include <ikf/Container/Timestamp.hpp>

class IKF_TMultiHistoryBuffer_test : public testing::Test
{
public:
};

ikf::TMultiHistoryBuffer<int> get_sorted_buffer_multi(int const n=10) {
  ikf::TMultiHistoryBuffer<int> hist;
  for (int i = 1; i <= n; i++) {
    hist.insert(i, ikf::Timestamp(i*0.1));
    hist.insert(-i, ikf::Timestamp(i*0.1));
  }
  return hist;
}

template <typename T>
void ctor_init_test() {
  ikf::TMultiHistoryBuffer<T> test;

  EXPECT_TRUE(test.size() == 0);
  EXPECT_TRUE(test.empty());
  EXPECT_FALSE(test.exist_at_t(ikf::Timestamp(0.2)));

  ikf::TStampedData<T> res;
  EXPECT_FALSE(test.at(1,res));

  test.insert(1, 0.1);
  test.insert(2, ikf::Timestamp(0.2));
  EXPECT_TRUE(test.exist_at_t(ikf::Timestamp(0.2)));
  test.insert(3, ikf::Timestamp(0.15));
  test.insert(4, ikf::Timestamp(0.4));
  test.insert(5, ikf::Timestamp(0.5));
  test.insert(6, ikf::Timestamp(0.45));
  EXPECT_TRUE(test.size() == 6);

  std::cout << test << std::endl;

  T elem = 0;
  EXPECT_TRUE(test.get_latest(elem));
  std::cout << "latest: " << elem << std::endl;
  EXPECT_TRUE(test.get_oldest(elem));
  std::cout << "oldest: " << elem << std::endl;

  EXPECT_FALSE(test.get_at_t(0.2222, elem));
  EXPECT_TRUE(test.get_at_t(0.2, elem));
  EXPECT_TRUE(elem == 2);
  std::cout << "at 0.2 sec: " << elem << std::endl;

  EXPECT_TRUE(test.get_at_t(ikf::Timestamp(0.45), elem));
  EXPECT_TRUE(elem == 6);
  std::cout << "at 0.45 sec: " << elem << std::endl;

  EXPECT_TRUE(test.get_before_t(ikf::Timestamp(0.45), elem));
  EXPECT_TRUE(elem == 4);
  std::cout << "before 0.45 sec: " << elem << std::endl;

  EXPECT_TRUE(test.get_after_t(ikf::Timestamp(0.45), elem));
  EXPECT_TRUE(elem == 5);
  std::cout << "after 0.45 sec: " << elem << std::endl;


  EXPECT_FALSE(test.get_before_t(ikf::Timestamp(0.1), elem));
  EXPECT_FALSE(test.get_after_t(ikf::Timestamp(0.5), elem));
}

TEST_F(IKF_TMultiHistoryBuffer_test, ctor_init)
{

  ctor_init_test<int>();
  ctor_init_test<double>();
  ctor_init_test<char>();


}

TEST_F(IKF_TMultiHistoryBuffer_test, insert)
{

  ikf::TMultiHistoryBuffer<int> test;

  EXPECT_TRUE(test.size() == 0);
  std::vector<int> elems = test.get_all_at_t(ikf::Timestamp(0.2));
  EXPECT_TRUE(elems.size() == 0);

  int val =0;
  test.insert(1, 0.1);
  test.insert(2, ikf::Timestamp(0.2));
  test.insert(3, ikf::Timestamp(0.15));
  EXPECT_TRUE(test.get_at_t(ikf::Timestamp(0.15), val));
  EXPECT_TRUE(val == 3);

  std::cout << test << std::endl;

  test.insert(4, ikf::Timestamp(0.15));
  std::cout << "insert 4 at 0.15 sec: " << std::endl;
  std::cout << test << std::endl;
  EXPECT_TRUE(test.size() == 4);

  EXPECT_TRUE(test.get_at_t(ikf::Timestamp(0.15), val));
  EXPECT_TRUE(val == 3);

  elems =test.get_all_at_t(ikf::Timestamp(0.15));
  EXPECT_TRUE(elems.size() == 2);
  EXPECT_TRUE(elems[0] == 3);
  EXPECT_TRUE(elems[1] == 4);

  elems = test.get_all_at_t(ikf::Timestamp(0.2));
  EXPECT_TRUE(elems.size() == 1);

  elems = test.get_all_at_t(ikf::Timestamp(0.3));
  EXPECT_TRUE(elems.size() == 0);
}
TEST_F(IKF_TMultiHistoryBuffer_test, erase_before)
{

  ikf::TMultiHistoryBuffer<int> test;

  EXPECT_TRUE(test.size() == 0);

  test.insert(1, 0.1);
  test.insert(2, ikf::Timestamp(0.2));
  test.insert(3, ikf::Timestamp(0.15));
  test.insert(4, ikf::Timestamp(0.4));
  test.insert(5, ikf::Timestamp(0.5));
  test.insert(6, ikf::Timestamp(0.45));
  EXPECT_TRUE(test.size() == 6);

  std::cout << test << std::endl;

  std::cout << "remove before 0.5\n";
  test.remove_before_t(ikf::Timestamp(0.5));
  std::cout << test << std::endl;
  EXPECT_TRUE(test.size() == 1);
}

TEST_F(IKF_TMultiHistoryBuffer_test, get_before)
{

  ikf::TMultiHistoryBuffer<int> test = get_sorted_buffer_multi(10);
  std::cout << test << std::endl;

  ikf::TStampedData<int> tdata;
  int data =0;
  ikf::Timestamp stamp;
  ikf::Timestamp t(0.5);
  EXPECT_TRUE(test.get_before_t(t, tdata));
  EXPECT_TRUE(test.get_before_t(t, data));
  EXPECT_TRUE(test.get_before_t(t, stamp));
  EXPECT_TRUE(tdata.data == -4);
  std::cout << "before t=0.5: tdata.stamp=" << tdata.stamp <<  ", tdata.data=" << tdata.data << std::endl;
  EXPECT_TRUE(tdata.stamp == ikf::Timestamp(0.4));
  EXPECT_TRUE(data == -4);
  EXPECT_TRUE(stamp == ikf::Timestamp(0.4));

  ikf::Timestamp t2(0.0);
  EXPECT_FALSE(test.get_before_t(t2, tdata));
  EXPECT_FALSE(test.get_before_t(t2, data));
  EXPECT_FALSE(test.get_before_t(t2, stamp));
}
TEST_F(IKF_TMultiHistoryBuffer_test, get_after)
{

  ikf::TMultiHistoryBuffer<int> test = get_sorted_buffer_multi(10);


  ikf::TStampedData<int> tdata;
  int data = 0;
  ikf::Timestamp stamp;
  ikf::Timestamp t(0.3);
  EXPECT_TRUE(test.get_after_t(t, tdata));
  EXPECT_TRUE(test.get_after_t(t, data));
  EXPECT_TRUE(test.get_after_t(t, stamp));
  EXPECT_TRUE(tdata.data == 4);
  EXPECT_TRUE(tdata.stamp == ikf::Timestamp(0.4));
  EXPECT_TRUE(data == 4);
  EXPECT_TRUE(stamp == ikf::Timestamp(0.4));


  ikf::Timestamp t2(1.0);
  EXPECT_FALSE(test.get_after_t(t2, tdata));
  EXPECT_FALSE(test.get_after_t(t2, data));
  EXPECT_FALSE(test.get_after_t(t2, stamp));
}


TEST_F(IKF_TMultiHistoryBuffer_test, erase_after)
{

  ikf::TMultiHistoryBuffer<int> test;

  EXPECT_TRUE(test.size() == 0);

  test.insert(1, 0.1);
  test.insert(2, ikf::Timestamp(0.2));
  test.insert(3, ikf::Timestamp(0.15));
  test.insert(4, ikf::Timestamp(0.4));
  test.insert(5, ikf::Timestamp(0.5));
  test.insert(6, ikf::Timestamp(0.45));
  EXPECT_TRUE(test.size() == 6);

  std::cout << test << std::endl;


  std::cout << "remove after 0.15\n";
  test.remove_after_t(ikf::Timestamp(0.15));
  std::cout << test << std::endl;
  EXPECT_TRUE(test.size() == 2);
}

TEST_F(IKF_TMultiHistoryBuffer_test, get_between) {
  ikf::TMultiHistoryBuffer<int> test = get_sorted_buffer_multi(10);

  std::cout << test << std::endl;
  ikf::TMultiHistoryBuffer<int> res = test.get_between_t1_t2(ikf::Timestamp(0.1), ikf::Timestamp(0.5));
  std::cout << "get_between_t1_t2: [0.1,0.5]\n";
  std::cout << res << std::endl;
  EXPECT_TRUE(res.size() == 5*2);

  ikf::TMultiHistoryBuffer<int> res2 = test.get_between_t1_t2(ikf::Timestamp(0.0), ikf::Timestamp(0.5));
  std::cout << "get_between_t1_t2: [0.0,0.5]\n";
  std::cout << res2 << std::endl;
  EXPECT_TRUE(res2.size() == 5*2);

  ikf::TMultiHistoryBuffer<int> res3 = test.get_between_t1_t2(ikf::Timestamp(0.7), ikf::Timestamp(0.5));
  std::cout << "get_between_t1_t2: [0.7,0.5]\n";
  std::cout << res3 << std::endl;
  EXPECT_TRUE(res3.size() == 0);

  ikf::TMultiHistoryBuffer<int> res4 = test.get_between_t1_t2(ikf::Timestamp(0.11), ikf::Timestamp(0.51));
  std::cout << "get_between_t1_t2: [0.11,0.51]\n";
  std::cout << res4 << std::endl;
  EXPECT_TRUE(res4.size() == 4*2);
}

TEST_F(IKF_TMultiHistoryBuffer_test, accum_between_sum) {

  ikf::TMultiHistoryBuffer<int> test = get_sorted_buffer_multi(10);
  std::cout << test << std::endl;

  std::cout << "sum between_t1_t2: [0.0,0.5]\n";
  int res = test.accumulate_between_t1_t2(ikf::Timestamp(0.0), ikf::Timestamp(0.5), 0, [](int const A, int const B){return B+A; });
  std::cout << res << std::endl;
  EXPECT_TRUE(res == 0);
}

TEST_F(IKF_TMultiHistoryBuffer_test, accum_sum) {

  ikf::TMultiHistoryBuffer<int> test = get_sorted_buffer_multi(10);
  std::cout << test << std::endl;

  std::cout << "sum:\n";
  int res = test.accumulate(0, [](int A, int B){return B+A; });
  std::cout << res << std::endl;
  EXPECT_TRUE(res == 0);


  std::cout << "sum between_t1_t2: [0.1,0.1]\n";
  res = test.accumulate_between_t1_t2(ikf::Timestamp(0.1), ikf::Timestamp(0.1), -1, [](int const A, int const B){return B+A; });
  std::cout << res << std::endl;
  EXPECT_TRUE(res == -1);

  std::cout << "sum between_t1_t2: [0.1,-0.0]\n";
  res = test.accumulate_between_t1_t2(ikf::Timestamp(0.1), ikf::Timestamp(0.0), -1, [](int const A, int const B){return B+A; });
  std::cout << res << std::endl;
  EXPECT_TRUE(res == -1);
}

TEST_F(IKF_TMultiHistoryBuffer_test, accum_between_prod) {

  ikf::TMultiHistoryBuffer<int> test = get_sorted_buffer_multi(10);
  std::cout << test << std::endl;

  std::cout << "prod between_t1_t2: [0.0,0.5]\n";
  int res = test.accumulate_between_t1_t2(ikf::Timestamp(0.0), ikf::Timestamp(0.5), 1, [](int A, int B){return B*A; });
  std::cout << res << std::endl;
  EXPECT_TRUE(res == -120*120);
}

TEST_F(IKF_TMultiHistoryBuffer_test, accum_prod) {

  ikf::TMultiHistoryBuffer<int> test = get_sorted_buffer_multi(5);
  std::cout << test << std::endl;

  std::cout << "prod:\n";
  int res = test.accumulate(1, [](int A, int B){return B*A; });
  std::cout << res << std::endl;
  EXPECT_TRUE(res == -120*120);
}


class Printer {
public:
  void print(int const i) {
    std::cout << "* val:" << i << std::endl;
  }

  void print_all()
  {
    ikf::Timestamp first, last;
    test.get_oldest_t(first);
    test.get_latest_t(last);
    test.foreach_between_t1_t2(first, last, [this](int const& i) { print(i);});
  }


  ikf::TMultiHistoryBuffer<int> test;
};

TEST_F(IKF_TMultiHistoryBuffer_test, foreach_between_t1_t2) {


  ikf::TMultiHistoryBuffer<int> test = get_sorted_buffer_multi(10);
  std::cout << test << std::endl;

  std::cout << "use foreach directly:" << std::endl;
  test.foreach([](int const& i){std::cout << "* i=" << i << std::endl;});

  std::cout << "use foreach_reverse directly:" << std::endl;
  test.foreach_reverse([](int const& i){std::cout << "* i=" << i << std::endl;});

  std::cout << "use printer class:" << std::endl;
  Printer printme;
  printme.test = test;
  printme.print_all();

}
