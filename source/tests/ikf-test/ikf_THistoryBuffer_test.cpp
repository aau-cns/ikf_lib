#include <gmock/gmock.h>
#include <ikf/Container/THistoryBuffer.hpp>
#include <ikf/Container/Timestamp.hpp>

class IKF_THistoryBuffer_test : public testing::Test
{
public:
};

ikf::THistoryBuffer<int> get_sorted_buffer(int const n=10) {
  ikf::THistoryBuffer<int> hist;
  for (int i = 1; i <= n; i++) {
    hist.insert(i, ikf::Timestamp(i*0.1));
  }
  return hist;
}

TEST_F(IKF_THistoryBuffer_test, ctor_init)
{

    ikf::THistoryBuffer<int> test;

    EXPECT_TRUE(test.size() == 0);

    test.insert(1, 0.1);
    test.insert(2, ikf::Timestamp(0.2));
    test.insert(3, ikf::Timestamp(0.15));
    test.insert(4, ikf::Timestamp(0.4));
    test.insert(5, ikf::Timestamp(0.5));
    test.insert(6, ikf::Timestamp(0.45));
    EXPECT_TRUE(test.size() == 6);

    std::cout << test << std::endl;

    int elem = 0;
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

TEST_F(IKF_THistoryBuffer_test, insert)
{

    ikf::THistoryBuffer<int> test;

    EXPECT_TRUE(test.size() == 0);
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
    EXPECT_TRUE(test.size() == 3);

    EXPECT_TRUE(test.get_at_t(ikf::Timestamp(0.15), val));
    EXPECT_TRUE(val == 4);
}
TEST_F(IKF_THistoryBuffer_test, erase_before)
{

    ikf::THistoryBuffer<int> test;

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

TEST_F(IKF_THistoryBuffer_test, get_before)
{

    ikf::THistoryBuffer<int> test = get_sorted_buffer(10);


    ikf::TStampedData<int> tdata;
    int data =0;
    ikf::Timestamp stamp;
    ikf::Timestamp t(0.5);
    EXPECT_TRUE(test.get_before_t(t, tdata));
    EXPECT_TRUE(test.get_before_t(t, data));
    EXPECT_TRUE(test.get_before_t(t, stamp));
    EXPECT_TRUE(tdata.data == 4);
    EXPECT_TRUE(tdata.stamp == ikf::Timestamp(0.4));
    EXPECT_TRUE(data == 4);
    EXPECT_TRUE(stamp == ikf::Timestamp(0.4));

    ikf::Timestamp t2(0.0);
    EXPECT_FALSE(test.get_before_t(t2, tdata));
    EXPECT_FALSE(test.get_before_t(t2, data));
    EXPECT_FALSE(test.get_before_t(t2, stamp));
}
TEST_F(IKF_THistoryBuffer_test, get_after)
{

    ikf::THistoryBuffer<int> test = get_sorted_buffer(10);


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


TEST_F(IKF_THistoryBuffer_test, erase_after)
{

    ikf::THistoryBuffer<int> test;

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

TEST_F(IKF_THistoryBuffer_test, get_between) {
  ikf::THistoryBuffer<int> test = get_sorted_buffer(10);

  std::cout << test << std::endl;
  ikf::THistoryBuffer<int> res = test.get_between_t1_t2(ikf::Timestamp(0.1), ikf::Timestamp(0.5));
  std::cout << "get_between_t1_t2: [0.1,0.5]\n";
  std::cout << res << std::endl;
  EXPECT_TRUE(res.size() == 5);

  ikf::THistoryBuffer<int> res2 = test.get_between_t1_t2(ikf::Timestamp(0.0), ikf::Timestamp(0.5));
  std::cout << "get_between_t1_t2: [0.0,0.5]\n";
  std::cout << res2 << std::endl;
  EXPECT_TRUE(res2.size() == 5);

  ikf::THistoryBuffer<int> res3 = test.get_between_t1_t2(ikf::Timestamp(0.7), ikf::Timestamp(0.5));
  std::cout << "get_between_t1_t2: [0.7,0.5]\n";
  std::cout << res3 << std::endl;
  EXPECT_TRUE(res3.size() == 0);

  ikf::THistoryBuffer<int> res4 = test.get_between_t1_t2(ikf::Timestamp(0.11), ikf::Timestamp(0.51));
  std::cout << "get_between_t1_t2: [0.11,0.51]\n";
  std::cout << res4 << std::endl;
  EXPECT_TRUE(res4.size() == 4);
}

TEST_F(IKF_THistoryBuffer_test, accum_between_sum) {

    ikf::THistoryBuffer<int> test = get_sorted_buffer(10);
    std::cout << test << std::endl;

    std::cout << "sum between_t1_t2: [0.0,0.5]\n";
    int res = test.accumulate_between_t1_t2(ikf::Timestamp(0.0), ikf::Timestamp(0.5), 0, [](int const A, int const B){return B+A; });
    std::cout << res << std::endl;
    EXPECT_TRUE(res == 15);
}

TEST_F(IKF_THistoryBuffer_test, accum_sum) {

    ikf::THistoryBuffer<int> test = get_sorted_buffer(10);
    std::cout << test << std::endl;

    std::cout << "sum:\n";
    int res = test.accumulate(0, [](int A, int B){return B+A; });
    std::cout << res << std::endl;
    EXPECT_TRUE(res == 55);
}

TEST_F(IKF_THistoryBuffer_test, accum_between_prod) {

    ikf::THistoryBuffer<int> test = get_sorted_buffer(10);
    std::cout << test << std::endl;

    std::cout << "prod between_t1_t2: [0.0,0.5]\n";
    int res = test.accumulate_between_t1_t2(ikf::Timestamp(0.0), ikf::Timestamp(0.5), 1, [](int A, int B){return B*A; });
    std::cout << res << std::endl;
    EXPECT_TRUE(res == 120);
}

TEST_F(IKF_THistoryBuffer_test, accum_prod) {

    ikf::THistoryBuffer<int> test = get_sorted_buffer(5);
    std::cout << test << std::endl;

    std::cout << "prod:\n";
    int res = test.accumulate(1, [](int A, int B){return B*A; });
    std::cout << res << std::endl;
    EXPECT_TRUE(res == 120);
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


    ikf::THistoryBuffer<int> test;
};

TEST_F(IKF_THistoryBuffer_test, foreach_between_t1_t2) {


    ikf::THistoryBuffer<int> test = get_sorted_buffer(10);
    std::cout << test << std::endl;

    std::cout << "use foreach directly:" << std::endl;
    test.foreach([](int const& i){std::cout << "* i=" << i << std::endl;});

    std::cout << "use printer class:" << std::endl;
    Printer printme;
    printme.test = test;
    printme.print_all();

}
