/******************************************************************************
* FILENAME:     IKF_TTHimeHorizonBuffer.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     20.01.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <gmock/gmock.h>
#include <ikf/container/TTimeHorizonBuffer.hpp>
#include <ikf/container/Timestamp.hpp>

class IKF_TTHimeHorizonBuffer_test : public testing::Test
{
public:
};

ikf::TTimeHorizonBuffer<int> get_sorted_hist_buffer(int const n=10) {
  ikf::TTimeHorizonBuffer<int> hist(n*0.1);
  for (int i = 1; i <= n; i++) {
    hist.insert(i, ikf::Timestamp(i*0.1));
  }
  return hist;
}

TEST_F(IKF_TTHimeHorizonBuffer_test, checkhorizon)
{

  std::cout << "Timestamp (0.0) " << ikf::Timestamp(0.0) << std::endl;
  std::cout << "Timestamp (-1.0) " << ikf::Timestamp(-1.0) << std::endl;
  ikf::TTimeHorizonBuffer<int> test = get_sorted_hist_buffer(10);

  std::cout << test << std::endl;
  std::cout << "horizon: " << test.horizon() << std::endl;
  EXPECT_NEAR(test.horizon(), 0.9, 0.001);

  std::cout << "set horizon to 0.5\n";
  test.set_horizon(0.5);
  test.check_horizon();
  std::cout << test << std::endl;
  std::cout << "horizon: " << test.horizon() << std::endl;
}



TEST_F(IKF_TTHimeHorizonBuffer_test, clone)
{


  ikf::TTimeHorizonBuffer<int> test = get_sorted_hist_buffer(10);

  auto test2 = test.clone();

  test2.insert(11, ikf::Timestamp(11.1));

  std::cout << "orig:" << test << std::endl;
  std::cout << "clone:" << test2 << std::endl;

}


