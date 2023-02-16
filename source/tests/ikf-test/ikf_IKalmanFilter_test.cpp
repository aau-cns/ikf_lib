/******************************************************************************
* FILENAME:     IKF_IKalmanFilter_test.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     25.01.2023
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
#include <ikf/Container/THistoryBuffer.hpp>
#include <ikf/Container/Timestamp.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/inverse_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>
#include <ikf/Estimator/NormalizedInnovationSquared.hpp>
#include <ikf/utils/eigen_utils.hpp>

double inverse_chi_squared_quantile(double df, double p)
{
  return quantile(boost::math::inverse_chi_squared(df), p);
}
double inverse_chi_squared_pdf(double df, double x)
{
  return pdf(boost::math::inverse_chi_squared(df), x);
}
double inverse_chi_squared_cdf(double df, double x)
{
  return cdf(boost::math::inverse_chi_squared(df), x);
}

double chi_squared_quantile(double df, double p)
{
  return quantile(boost::math::chi_squared(df), p);
}


class IKF_IKalmanFilter_test : public testing::Test
{
public:
};


TEST_F(IKF_IKalmanFilter_test, ctor_init)
{
  double df = 1;
  double p = 0.997;

  std::cout << "df=" << df << ", p=" << p << std::endl;
  std::cout << "inverse_chi_squared_quantile(df,  p)" << inverse_chi_squared_quantile(df, p) << std::endl;
  std::cout << "inverse_chi_squared_pdf( df,  p)" << inverse_chi_squared_pdf(df, p) << std::endl;
  std::cout << "inverse_chi_squared_cdf( df,  p)" << inverse_chi_squared_cdf(df, p) << std::endl;
  std::cout << "chi_squared_quantile( df,  p)" << chi_squared_quantile(df, p) << std::endl;

}

TEST_F(IKF_IKalmanFilter_test, NIS_table)
{

  for (auto const& elem : ikf::NormalizedInnovationSquared::lookup) {
    std::cout << "0." << elem.first << ": ";
    for (auto const& itim : elem.second) {
      std::cout << itim << ", ";
    }
    std::cout << std::endl;
  }


}

TEST_F(IKF_IKalmanFilter_test, Eigen)
{
  Eigen::MatrixXd mat = Eigen::MatrixXd::Random(3,3);

  std::cout << mat << std::endl;
  std::cout << mat.Identity(mat.rows(), mat.cols()) << std::endl;
  std::cout << mat << std::endl;


  auto mat_sym = ikf::utils::symmetrize_covariance(mat);
  std::cout << mat_sym << std::endl;
  EXPECT_TRUE(mat_sym.isApprox(mat_sym.transpose().eval()));
}

TEST_F(IKF_IKalmanFilter_test, negative_diag)
{
  Eigen::Vector3d v(1, 2, 3);
  EXPECT_FALSE(ikf::utils::negative_diag(v.asDiagonal()));
  Eigen::Vector3d v2(0, 0, 0);
  EXPECT_FALSE(ikf::utils::negative_diag(v2.asDiagonal()));
  Eigen::Vector3d v3(1, 2, -0.0001);
  Eigen::MatrixXd mat =  v3.asDiagonal();
  std::cout << mat << std::endl;
  EXPECT_TRUE(ikf::utils::negative_diag(v3.asDiagonal()));

}

TEST_F(IKF_IKalmanFilter_test, is_positive_semidefinite)
{
  Eigen::Vector3d v(1, 2, 3);
  EXPECT_TRUE(ikf::utils::is_positive_semidefinite(v.asDiagonal()));

  Eigen::Matrix3d A;
  A << 3, 9, 1,
      9, 3, 2,
      1, 2, 3;
  EXPECT_FALSE(ikf::utils::is_positive_semidefinite(A));

  // generated in Matlab with: A = PSD.random(3);
  Eigen::Matrix3d B;
  B <<  3.4441,   -0.4847,    0.3928,
      -0.4847,    0.7365,    0.0991,
      0.3928,    0.0991,    1.5343;
  EXPECT_TRUE(ikf::utils::is_positive_semidefinite(B));
}
