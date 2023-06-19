/******************************************************************************
* FILENAME:     MeasDataConverter.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         roland.jung@ieee.org
* VERSION:      v0.0.1
* CREATION:     13.06.2023
*
*  Copyright (C) 2023
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef IKF_MEASDATACONVERTER_HPP
#define IKF_MEASDATACONVERTER_HPP
#include <ikf/ikf_api.h>
#include <Eigen/Dense>
#include <ikf/utils/eigen_utils.hpp>

namespace ikf {

class IKF_API MeasDataConverter
{
public:

  static Eigen::VectorXd to_IMU_data(Eigen::Vector3d const a_I, Eigen::Vector3d const w_I) {
    return utils::vertcat_vec(a_I, w_I);
  }

  static void from_IMU_data(Eigen::VectorXd const z, Eigen::Vector3d & a_I, Eigen::Vector3d& w_I) {
    a_I << z({0,1,2});
    w_I << z({3,4,5});
  }

  static Eigen::VectorXd to_WHEEL_ODOM_CMD_data(Eigen::Vector3d const v_B, Eigen::Vector3d const w_B) {
    return utils::vertcat_vec(v_B, w_B);
  }

  static void from_WHEEL_ODOM_CMD_data(Eigen::VectorXd const z, Eigen::Vector3d & v_B, Eigen::Vector3d & w_B) {
    v_B << z({0,1,2});
    w_B << z({3,4,5});
  }


  static Eigen::VectorXd to_POSE_data(Eigen::Vector3d const p, Eigen::Quaterniond const q) {
    Eigen::Vector4d q_v;
    q_v << q.w(), q.x(), q.y(), q.z();
    return utils::vertcat_vec(p, q_v);
  }

  static void from_POSE_data(Eigen::VectorXd const z, Eigen::Vector3d &p, Eigen::Quaterniond &q) {
    q = Eigen::Quaterniond(z(3), z(4), z(5), z(6));
    p << z(0), z(1), z(2);
  }

  static Eigen::VectorXd to_POS_data(Eigen::Vector3d const p) {
    return Eigen::VectorXd(p);
  }

  static void from_POS_data(Eigen::VectorXd const z, Eigen::Vector3d &p) {
    p << z(0), z(1), z(2);
  }

  static Eigen::VectorXd to_RANGE_data(double const range, size_t const subject_id = 0)
  {
    Eigen::VectorXd z;
    z << range, 1.0*subject_id;
    return z;
  }

  static void from_RANGE_data(Eigen::VectorXd const z,double & range, size_t & subject_id)
  {
    range = z(0);
    subject_id = (size_t)z(1);
  }

  static Eigen::VectorXd to_BEARING_data(double const azimuth, double const elevation,  size_t const subject_id = 0)
  {
    Eigen::VectorXd z;
    z << azimuth, elevation, 1.0*subject_id;
    return z;
  }

  static void from_BEARING_data(Eigen::VectorXd const z,double & azimuth, double &elevation,  size_t & subject_id)
  {
    azimuth = z(0);
    elevation = z(1);
    subject_id = (size_t)z(2);
  }


  static Eigen::VectorXd to_SPHERICAL_data(double const azimuth, double const elevation, double const range, size_t const subject_id = 0)
  {
    Eigen::VectorXd z;
    z << azimuth, elevation, range, 1.0*subject_id;
    return z;
  }

  static void from_SPHERICALG_data(Eigen::VectorXd const z,double & azimuth, double &elevation,  double& range, size_t & subject_id)
  {
    azimuth = z(0);
    elevation = z(1);
    range = z(2);
    subject_id = (size_t)z(3);
  }

  static Eigen::VectorXd to_GPS_data(double const latitude, double const longitude, double const altitude)
  {
    Eigen::VectorXd z;
    z << latitude, longitude, altitude;
    return z;
  }

  static void from_GPS_data(Eigen::VectorXd const z,double & latitude, double &longitude,  double& altitude)
  {
    latitude = z(0);
    longitude = z(1);
    altitude = z(2);
  }

  static Eigen::VectorXd to_GPSVel_data(double const latitude, double const longitude, double const altitude, Eigen::Vector3d const v_GS_G)
  {
    Eigen::VectorXd z;
    z << latitude, longitude, altitude, v_GS_G;
    return z;
  }

  static void from_GPSVel_data(Eigen::VectorXd const z,double & latitude, double &longitude,  double& altitude, Eigen::Vector3d& v_GS_G)
  {
    latitude = z(0);
    longitude = z(1);
    altitude = z(2);
    v_GS_G = z({3,4,5});
  }

};

}

#endif // MEASDATACONVERTER_HPP
