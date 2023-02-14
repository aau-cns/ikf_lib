/******************************************************************************
* FILENAME:     Trajectory.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.02.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
#include <iostream>
#include <ikf/utils/eigen_utils.hpp>
#include <ikf/utils/eigen_mvn.hpp>
#include <ikf/utils/RTVerification.hpp>
#include <matplot/matplot.h>

class Trajectory {
public:
  Eigen::ArrayXd p_arr;
  Eigen::ArrayXd v_arr;
  Eigen::ArrayXd a_arr;
  Eigen::ArrayXd t_arr;

  Trajectory(size_t const n=0) {
    if (n > 0) {
      p_arr.setZero(n,1);
      v_arr.setZero(n,1);
      a_arr.setZero(n,1);
      t_arr.setZero(n,1);
    }
  }
  size_t size() {
    return t_arr.size();
  }


  void print() {
    std::cout << "t_arr=" << t_arr.transpose() << std::endl;
    std::cout << "p_arr=" << p_arr.transpose() << std::endl;
    std::cout << "v_arr=" << v_arr.transpose() << std::endl;
    std::cout << "a_arr=" << a_arr.transpose() << std::endl;
  }

  void plot_trajectory(size_t const num_fig=0) {
    using namespace matplot;
    auto f1 = figure(true);
    f1->name("Trajectory" + std::to_string(num_fig));
    f1->number_title(true);
    f1->draw();
    f1->font("Arial");
    f1->font_size(12);
    f1->title("Fig" + std::to_string(num_fig));

    tiledlayout(3, 1);
    std::vector<double> x = ikf::utils::to_vector(t_arr);
    std::vector<double> p = ikf::utils::to_vector(p_arr);
    std::vector<double> v = ikf::utils::to_vector(v_arr);
    std::vector<double> a = ikf::utils::to_vector(a_arr);
    auto ax1 = nexttile();
    plot(ax1, x, p);
    ylabel(ax1, "p");
    xlabel(ax1, "t_{seconds}");
    grid(ax1, on);

    auto ax2 = nexttile();
    plot(ax2, x, v);
    ylabel(ax2, "v");
    xlabel(ax2, "t_{seconds}");
    grid(ax2, on);
    auto ax3 = nexttile();
    plot(ax3, x, a);
    ylabel(ax3, "a");
    xlabel(ax3, "t_{seconds}");
    grid(ax3, on);

    f1->draw();
    save("img/barchart_jpeg", "jpeg");
  }

  Eigen::Array<double, Eigen::Dynamic, 1> generate_pos_meas(double const std_dev_p) {
    //Eigen::UnivariateNormal<double> normX_solver(0,std_dev_p);
    Eigen::Array<double, Eigen::Dynamic, 1> res;
    res.setZero(10,1);
    return res;
  }

  Eigen::Array<double, Eigen::Dynamic, 1> generate_acc_meas(double const std_dev_a) {
    Eigen::Array<double, Eigen::Dynamic, 1> res;
    res.setZero(10,1);
    return res;
  }


  Eigen::ArrayXd generate_noisy_pos(double const std_dev) {
    ikf::UnivariateNormal<double> gen1(0, std_dev);
    size_t n_samples = t_arr.size();
    Eigen::MatrixXd noise_arr =   gen1.samples(n_samples).transpose();
    return noise_arr.array() + p_arr;
  }

  Eigen::ArrayXd generate_noisy_vel(double const std_dev) {
    ikf::UnivariateNormal<double> gen1(0, std_dev);
    size_t n_samples = t_arr.size();
    Eigen::MatrixXd noise_arr =   gen1.samples(n_samples).transpose();
    return noise_arr.array() + v_arr;
  }

  Eigen::ArrayXd generate_noisy_acc(double const std_dev) {
    ikf::UnivariateNormal<double> gen1(0, std_dev);
    size_t n_samples = t_arr.size();
    Eigen::MatrixXd noise_arr =   gen1.samples(n_samples).transpose();
    return noise_arr.array() + a_arr;
  }



  void generate_sine(double const dt, double const D, double const omega,
                     double const omega_0, double const amplitude, double const offset = 0) {

    //t_arr = Eigen::Map<Eigen::Array<double, Eigen::Dynamic, 1>, Eigen::Unaligned>(ls.data(), ls.size());
    size_t const N = ((D)/dt) + 1;
    t_arr.setLinSpaced(N, 0, D);
    p_arr = Eigen::sin(t_arr*omega   + omega_0)*amplitude + offset;
    v_arr = omega*Eigen::cos(t_arr*omega   + omega_0)*amplitude;
    a_arr = -omega*omega*Eigen::sin(omega*t_arr  + omega_0)*amplitude;

  }

  Eigen::ArrayXd generate_noisy_rel_pos(Trajectory const& other, double const std_dev) {
    ikf::RTV_EXPECT_TRUE_THROW(t_arr.size() == other.t_arr.size(), "Dimension missmatch");
    Eigen::ArrayXd res = t_arr - other.t_arr;
    ikf::RTV_EXPECT_TRUE_THROW(res.sum() < 0.1, "Time indices mismacht!");
    ikf::UnivariateNormal<double> gen1(0, std_dev);
    size_t n_samples = t_arr.size();
    Eigen::MatrixXd noise_arr =   gen1.samples(n_samples).transpose();
    return  other.p_arr  - p_arr + noise_arr.array();
  }



  static std::vector<double> linespace(double const start, double const step, double const stop) {
    size_t const N = ((stop-start)/step) + 1;
    std::vector<double> xs(N);
    double val = start;
    for(auto x = xs.begin(); x != xs.end(); ++x, val += step) {
      *x = val;
    }
    return xs;
  }
};

#endif // TRAJECTORY_HPP
