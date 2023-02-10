#include <gmock/gmock.h>
#include <ikf/container/TBuffer.hpp>
#include <ikf/container/Timestamp.hpp>

class IKF_TBuffer_test : public testing::Test
{
  public:
};

class derived_from_timestamped : public ikf::Timestamped
{
  public:
};

TEST_F(IKF_TBuffer_test, ctor_init)
{
  ikf::TBuffer<derived_from_timestamped> test;

  EXPECT_TRUE(test.get_max_buffer_size() == 256);
}

TEST_F(IKF_TBuffer_test, set_get_max_buffer_size)
{
  ikf::TBuffer<derived_from_timestamped> test;

  unsigned test_size = 10;
  test.set_max_buffer_size(test_size);
  EXPECT_TRUE(test.get_max_buffer_size() == test_size);

  test_size = 15;
  test.set_max_buffer_size(test_size);
  EXPECT_TRUE(test.get_max_buffer_size() == test_size);
}

TEST_F(IKF_TBuffer_test, insert_one_get_size)
{
  constexpr unsigned test_size_max = 100;

  ikf::TBuffer<derived_from_timestamped> test;
  test.set_max_buffer_size(test_size_max);

  EXPECT_TRUE(test.get_size() == 0);

  derived_from_timestamped time_test;
  time_test.from_sec(123.4);
  test.insert_sorted(time_test);
  EXPECT_TRUE(test.get_size() == 1);
}

TEST_F(IKF_TBuffer_test, insert_multiple_get_size)
{
  constexpr unsigned test_size = 100;
  constexpr unsigned test_size_max = 110;

  ikf::TBuffer<derived_from_timestamped> test;
  test.set_max_buffer_size(test_size_max);

  derived_from_timestamped time_test;

  for(unsigned i=0; i<test_size; i++)
  {
    time_test.from_sec(double(i));
    test.insert_sorted(time_test);
  }

  EXPECT_TRUE(test.get_size() == test_size);
}

TEST_F(IKF_TBuffer_test, get_buffer_at_t_relative_out_of_range)
{
  ikf::TBuffer<derived_from_timestamped> test_buffer;
  derived_from_timestamped time_test;
  derived_from_timestamped time_result;
  unsigned sample_count = 10;

  for(unsigned i = 0; i<= sample_count; i++)
  {
    derived_from_timestamped time_test;
    time_test.from_sec(double(i));

    test_buffer.insert_sorted(time_test);
  }

  time_test.from_sec(0.0);

  EXPECT_FALSE(test_buffer.get_buffer_at_t_relative(time_test, -1, time_result));
  EXPECT_TRUE(test_buffer.get_buffer_at_t_relative(time_test, sample_count-1, time_result));
}

TEST_F(IKF_TBuffer_test, get_buffer_at_t_relative_empty)
{
  ikf::TBuffer<derived_from_timestamped> test_buffer;
  derived_from_timestamped time_test;
  derived_from_timestamped time_result;

  time_test.from_sec(0.0);

  EXPECT_FALSE(test_buffer.get_buffer_at_t_relative(time_test, 0, time_result));
}


TEST_F(IKF_TBuffer_test, get_buffer_at_t_relative_access)
{
  const std::vector<double> test_time_dataset
  {
    12.1234,
    13.654,
    14.976,
    15.234,
    16.234,
    17.793,
    18.3,
    18.5,
    18.54,
    19.0
  };

  ikf::TBuffer<derived_from_timestamped> test_buffer;

  const unsigned test_size = 100;
  test_buffer.set_max_buffer_size(test_size);

  derived_from_timestamped time_test_class;

  for(unsigned i = 0; i< test_time_dataset.size(); i++)
  {
    time_test_class.from_sec(double(test_time_dataset[i]));

    test_buffer.insert_sorted(time_test_class);
  }

  derived_from_timestamped time_result;
  derived_from_timestamped time_test;
  time_test.from_sec(16.234);

  EXPECT_TRUE(test_buffer.get_buffer_at_t_relative(time_test, -1, time_result));
  EXPECT_TRUE(time_result.to_sec() == 15.234);
  EXPECT_TRUE(test_buffer.get_buffer_at_t_relative(time_test, 1, time_result));
  EXPECT_TRUE(time_result.to_sec() == 17.793);
}

TEST_F(IKF_TBuffer_test, insert_sorted)
{
  const std::vector<double> test_time_dataset
  {
    12.1234,
    13.654,
    14.976,
    15.234,
    16.234,
    17.793,
    18.3,
    18.5,
    18.54,
    19.0
  };

  ikf::TBuffer<derived_from_timestamped> test_buffer;

  const unsigned test_size = 100;
  test_buffer.set_max_buffer_size(test_size);

  derived_from_timestamped time_test_class;

  for(unsigned i = 0; i< test_time_dataset.size(); i++)
  {
    time_test_class.from_sec(double(test_time_dataset[i]));

    test_buffer.insert_sorted(time_test_class);
  }

  derived_from_timestamped out_of_order_timestamp;

  out_of_order_timestamp.from_sec(10.0);
  test_buffer.insert_sorted(out_of_order_timestamp);

  out_of_order_timestamp.from_sec(15.0);
  test_buffer.insert_sorted(out_of_order_timestamp);

  out_of_order_timestamp.from_sec(20.0);
  test_buffer.insert_sorted(out_of_order_timestamp);

  derived_from_timestamped out_of_order_result;

  const std::vector<double> correct_result
  {
    10.0, //inserted element
    12.1234,
    13.654,
    14.976,
    15.0, //inserted element
    15.234,
    16.234,
    17.793,
    18.3,
    18.5,
    18.54,
    19.0,
    20.0
  };  //inserted element

  std::vector<double> resulting_buffer;
  resulting_buffer.reserve(correct_result.size());

  time_test_class.from_sec(double(correct_result[0]));

  for(unsigned i = 0; i < test_buffer.get_size(); i++)
  {
    test_buffer.get_buffer_at_t_relative(time_test_class,i, out_of_order_result);
    resulting_buffer.push_back(out_of_order_result.to_sec());
  }

  EXPECT_TRUE(correct_result.size() == resulting_buffer.size());
  EXPECT_TRUE(correct_result == resulting_buffer);
}


TEST_F(IKF_TBuffer_test, get_oldest)
{
  ikf::TBuffer<derived_from_timestamped> test;

  // empty buffer access trows warning
  derived_from_timestamped elem;
  EXPECT_FALSE(test.get_oldest(elem));

  for(unsigned i = 1; i<= 10; i++)
  {
    derived_from_timestamped time_test;
    time_test.from_sec(double(i));

    test.insert_sorted(time_test);
  }

  EXPECT_TRUE(test.get_oldest(elem));
  EXPECT_TRUE(elem.to_sec() == 1);
}

TEST_F(IKF_TBuffer_test, get_latest)
{
  ikf::TBuffer<derived_from_timestamped> test;

  // empty buffer access trows warning
  derived_from_timestamped elem;
  EXPECT_FALSE(test.get_latest(elem));

  for(unsigned i = 1; i<= 10; i++)
  {
    derived_from_timestamped time_test;
    time_test.from_sec(double(i));

    test.insert_sorted(time_test);
  }
  EXPECT_TRUE(test.get_latest(elem));
  EXPECT_TRUE(elem.to_sec() == 10);
}

TEST_F(IKF_TBuffer_test, is_newer)
{
  ikf::TBuffer<derived_from_timestamped> test;

  // empty buffer access trows warning
  derived_from_timestamped elem;
  elem.from_sec(0.0);
  EXPECT_TRUE(test.is_newer(elem));
  for(unsigned i = 1; i<= 10; i++)
  {
    derived_from_timestamped time_test;
    time_test.from_sec(double(i));

    test.insert_sorted(time_test);
  }

  elem.from_sec(1);
  EXPECT_FALSE(test.is_newer(elem));

  elem.from_sec(11);
  EXPECT_TRUE(test.is_newer(elem));
}


TEST_F(IKF_TBuffer_test, max_buffer_overflow)
{
  ikf::TBuffer<derived_from_timestamped> test;

  const unsigned test_size = 10;
  test.set_max_buffer_size(test_size);


  derived_from_timestamped time_test;

  for(unsigned i = 0; i< 50; i++)
  {
    derived_from_timestamped time_test;
    time_test.from_sec(double(i));

    test.insert_sorted(time_test);
  }

  EXPECT_TRUE(test.get_max_buffer_size() == test_size);
}

TEST_F(IKF_TBuffer_test, get_buffer_at_t_empty)
{
  ikf::TBuffer<derived_from_timestamped> test_buffer;
  derived_from_timestamped time_test;
  derived_from_timestamped time_result;

  time_test.from_sec(0.0);

  EXPECT_FALSE(test_buffer.get_buffer_at_t(time_test, time_result));
}

TEST_F(IKF_TBuffer_test, get_buffer_at_t_correct_entries)
{
  std::vector<double> test_time_dataset
  {
    12.1234,
    13.654,
    14.976,
    15.234,
    16.234,
    17.793,
    18.3,
    18.5,
    18.54,
    19.0
  };

  ikf::TBuffer<derived_from_timestamped> test_buffer;

  const int test_size = 1000;
  test_buffer.set_max_buffer_size(test_size);

  derived_from_timestamped time_test_class;

  for(unsigned i = 0; i< test_time_dataset.size(); i++)
  {
    time_test_class.from_sec(double(test_time_dataset[i]));

    test_buffer.insert_sorted(time_test_class);
  }

  derived_from_timestamped buffer_get_test_result;
  ikf::Timestamp test_sample_t;

  // Exact matching stamp
  test_sample_t.from_sec(18.3);
  EXPECT_TRUE(test_buffer.get_buffer_at_t(test_sample_t, buffer_get_test_result));

  EXPECT_TRUE(buffer_get_test_result.to_sec() == 18.3);

  // Equal distance between two stamps
  // Return more current one
  test_sample_t.from_sec(18.41);
  EXPECT_TRUE(test_buffer.get_buffer_at_t(test_sample_t, buffer_get_test_result));

  EXPECT_TRUE(buffer_get_test_result.to_sec() == 18.5);

  // Closer distance too later stamp
  test_sample_t.from_sec(18.31);
  EXPECT_TRUE(test_buffer.get_buffer_at_t(test_sample_t, buffer_get_test_result));

  EXPECT_TRUE(buffer_get_test_result.to_sec() == 18.3);

  // Read index 0 as well, latest stamp
  test_sample_t.from_sec(12.1234);
  EXPECT_TRUE(test_buffer.get_buffer_at_t(test_sample_t, buffer_get_test_result));

  EXPECT_TRUE(buffer_get_test_result.to_sec() == 12.1234);
}
