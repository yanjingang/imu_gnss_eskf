#ifndef UTILS_H
#define UTILS_H

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeographicLib/LocalCartesian.hpp>
#include <iostream>
#include <fstream>
#include <deque>
#include <cfloat>

constexpr double D_R = M_PI / 180.;
constexpr double R_D = 180. / M_PI;
constexpr double g = 9.81007;
constexpr int IMU_Buffer_Size = 100;
constexpr double IMU_Std = 3.0;

struct IMUData{
    double timestamp;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};
using IMUDataPtr = std::shared_ptr<IMUData>;

struct GNSSData
{
    double timestamp;

    Eigen::Vector3d lla;
    Eigen::Matrix3d cov;
};
using GNSSDataPtr = std::shared_ptr<GNSSData>;

struct State{
    double timestamp;

    Eigen::Vector3d p_G_I;      // odom_msg.pose.pose translation
    Eigen::Vector3d v_G_I;      // odom_msg.twist.twist.linear
    Eigen::Matrix3d R_G_I;      // odom_msg.pose.pose linear
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d angular;    // odom_msg.twist.twist.angular

    Eigen::Matrix<double, 15, 15> cov;

};
using StatePtr = std::shared_ptr<State>;


inline void convert_lla_to_enu(const Eigen::Vector3d& init_lla,
                            const Eigen::Vector3d& point_lla,
                            Eigen::Vector3d* point_enu) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2),
                            point_enu->data()[0], point_enu->data()[1], point_enu->data()[2]);
}

inline void convert_enu_to_lla(const Eigen::Vector3d& init_lla,
                            const Eigen::Vector3d& point_enu,
                            Eigen::Vector3d* point_lla) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2),
                            point_lla->data()[0], point_lla->data()[1], point_lla->data()[2]);
}

inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;

    return w;
}

inline Eigen::Matrix3d v_expmap(Eigen::Vector3d x){
    Eigen::Vector3d w;
    double theta, theta2, theta3;
    Eigen::Matrix3d W, I, V;
    w << x(0), x(1), x(2);
    theta = w.norm();   theta2 = theta*theta; theta3 = theta2*theta;
    W = skew_matrix(w);
    I << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    if(theta>0.00000001)
        V << I + ((1-cos(theta))/theta2)*W + ((theta-sin(theta))/theta3)*W*W;
    else
        V << I;
    return V;
}

#endif // UTILS_H
