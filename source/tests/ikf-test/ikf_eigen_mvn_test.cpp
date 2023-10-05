/******************************************************************************
* FILENAME:     eigen_mvn_test.cpp
* PURPOSE:      Part of the ikf_lib
* AUTHOR:       Roland Jung
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     04.02.2023
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
#include <fstream>
#include <gmock/gmock.h>
#include <ikf/utils/MultivariateNormal.hpp>
#include <ikf/utils/UnivariateNormal.hpp>
#include <iostream>
#include <math.h>

#ifndef M_PI
#define M_PI (double)(3.1415926535897932384626433832795029)
#endif

/**
  Take a pair of un-correlated variances.
  Create a covariance matrix by correlating
  them, sandwiching them in a rotation matrix.
*/
Eigen::Matrix2d genCovar(double const v0=1.0, double const v1=1.0, double const theta=0.0)
{
  Eigen::Matrix2d rot = Eigen::Rotation2Dd(theta).matrix();

  Eigen::Matrix2d res = rot*Eigen::DiagonalMatrix<double,2,2>(v0,v1)*rot.transpose();
  return res;
}


class eigen_mvn_test : public testing::Test
{
public:
};

TEST_F(eigen_mvn_test, univariate)
{
  int const n_samples = 10;

  ikf::UnivariateNormal<double> gen1(0, 1.0);
  Eigen::MatrixXd samples1 =   gen1.samples(n_samples);
  std::cout << "samples1 " << samples1 << std::endl;


  ikf::UnivariateNormal<double> gen2(0, 2.0);
  gen2.setSeed(1234);
  Eigen::MatrixXd samples2 =   gen2.samples(n_samples);

  std::cout << "samples2 " << samples2 << std::endl;

  ikf::UnivariateNormal<double> gen3(0, 2.0);
  gen3.setSeed(1234);
  Eigen::MatrixXd samples3 =   gen3.samples(n_samples);
  std::cout << "samples3 " << samples3 << std::endl;
}


//TEST_F(eigen_mvn_test, ctor)
//{
//  Eigen::Vector2d mean;
//  Eigen::Matrix2d covar;
//  mean << -1,0.5; // Set the mean
//  // Create a covariance matrix
//  // Much wider than it is tall
//  // and rotated clockwise by a bit
//  covar = genCovar(3.0,0.1,M_PI/5.0);

//  // Create a bivariate gaussian distribution of doubles.
//  // with our chosen mean and covariance
//  const int dim = 2;
//  Eigen::MultivariateNormal<double> normX_solver(mean,covar);
//  std::ofstream file_solver("samples_solver.txt");

//  // Generate some samples and write them out to file
//  // for plotting
//  file_solver << normX_solver.samples(5000).transpose() << std::endl;
//  file_solver.close();
//  // same for Cholesky decomposition.
//  covar = genCovar(3,0.1,M_PI/5.0);
//  Eigen::MultivariateNormal<double> normX_cholesk(mean,covar,true);
//  std::ofstream file_cholesky("samples_cholesky.txt");
//  file_cholesky << normX_cholesk.samples(5000).transpose() << std::endl;
//  file_cholesky.close();

//  Eigen::Vector3d mean_3;
//  mean_3 << 1, 0, -1.4;
//  Eigen::Matrix3d cov3 = Eigen::Matrix3d::Identity();
//  Eigen::MultivariateNormal<double> normX_solver3(mean_3,cov3);
//  std::ofstream file_solver3("samples_solver3.txt");

//  int const n_samples = 1000;

//  Eigen::MatrixXd samples3 =   normX_solver3.samples(n_samples).transpose();
//  file_solver3 << samples3 << std::endl;
//  file_solver3.close();
//  Eigen::Vector3d Mean3 =samples3.colwise().mean();
//  std::cout << "mean3 (in)=" << mean_3 << ", \n sample Mean3 (result)=" << Mean3 <<   std::endl;

//  Eigen::MatrixXd samples3_zm = samples3.rowwise() - Mean3.transpose();

//  if (n_samples < 20) {
//    std::cout << "samples3 " << samples3 << std::endl;
//    std::cout << "samples3_zm " << samples3_zm << std::endl;
//  }
//  Eigen::Matrix3d Cov3 = (samples3_zm.transpose() * samples3_zm) * (1.0/((double)n_samples-1.0));
//  std::cout << "cov3 (in)=" << cov3 << ", \n sample Cov3 (result)=" << Cov3 << std::endl;

//}


TEST_F(eigen_mvn_test, seed)
{
  Eigen::Vector2d mean;
  Eigen::Matrix2d covar = genCovar(3.0,0.1,M_PI/5.0);
  mean << -1,0.5; // Set the mean

  int const n_samples = 10;

  ikf::MultivariateNormal<double> gen1(mean,covar);
  Eigen::MatrixXd samples1 =   gen1.samples(n_samples);
  std::cout << "samples1 " << samples1 << std::endl;

  ikf::MultivariateNormal<double> gen2(mean,covar);
  Eigen::MatrixXd samples2 =   gen2.samples(n_samples);
  std::cout << "samples2 " << samples2 << std::endl;

  ikf::MultivariateNormal<double> gen3(mean,covar);
  gen3.setSeed(1234); // std::abs(std::rand()));
  Eigen::MatrixXd samples3 =   gen3.samples(n_samples);
  std::cout << "samples3 " << samples3 << std::endl;

  ikf::MultivariateNormal<double> gen4(mean,covar);
  gen4.setSeed(1234); // std::abs(std::rand()));
  Eigen::MatrixXd samples4 =   gen4.samples(n_samples);
  std::cout << "samples4 " << samples4 << std::endl;
}


