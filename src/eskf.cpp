
#include "../include/eskf.h"
ESKF::ESKF(double acc_n, double gyro_n, double acc_w, double gyro_w, Eigen::Vector3d p_IMU_GNSS)
{
    acc_noise_ = acc_n;
    gyro_noise_ = gyro_n;
    acc_bias_noise_ = acc_w;
    gyro_bias_noise_ = gyro_w;
    p_I_GNSS_ = p_IMU_GNSS;
    state_ptr_ = std::make_shared<State>();
    initialized_ = false;
}
bool ESKF::process_IMU_Data(IMUDataPtr imu_data_ptr)
{
    if(!initialized_)
    {
        imu_buffer_.push_back(imu_data_ptr);
        if(imu_buffer_.size() > IMU_Buffer_Size) imu_buffer_.pop_front();
        return false;
    }

    predict(last_imu_ptr_, imu_data_ptr);
    last_imu_ptr_ = imu_data_ptr;
    return true;
}

void ESKF::predict(IMUDataPtr last_imu_ptr, IMUDataPtr cur_imu_ptr)
{
    double dt = cur_imu_ptr->timestamp - last_imu_ptr->timestamp;
    double dt_2 = dt * dt;

    State last_state = *state_ptr_;

    // timestamp
    state_ptr_->timestamp = cur_imu_ptr->timestamp;

    // p v R
    Eigen::Vector3d acc_unbias = 0.5 * (last_imu_ptr->acc + cur_imu_ptr->acc) - last_state.acc_bias;
    Eigen::Vector3d gyr_unbias = 0.5 * (last_imu_ptr->gyro + cur_imu_ptr->gyro) - last_state.gyro_bias;
    Eigen::Vector3d acc_nominal = last_state.R_G_I * acc_unbias + Eigen::Vector3d(0., 0., -g);
    state_ptr_->p_G_I = last_state.p_G_I + last_state.v_G_I * dt + 0.5 * acc_nominal * dt_2;
    state_ptr_->v_G_I = last_state.v_G_I + acc_nominal * dt;
    state_ptr_->angular = last_state.angular + gyr_unbias * dt;
    Eigen::Vector3d delta_angle_axis = gyr_unbias * dt;
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    if(delta_angle_axis.norm() > DBL_EPSILON)
    {
        dR = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
        state_ptr_->R_G_I = last_state.R_G_I * dR;
    }

    //Fx =
    //[I    I*dt    0    0    0]
    //[0    I -R[a]^dt -Rdt   0]
    //[0    0  Rt{w*dt}  0  Idt]
    //[0    0       0    I    0]
    //[0    0       0    0    I]

    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(3, 6) = -state_ptr_->R_G_I * skew_matrix(acc_unbias) * dt;
    Fx.block<3, 3>(3, 9) = -state_ptr_->R_G_I * dt;
    if (delta_angle_axis.norm() > DBL_EPSILON) {
        Fx.block<3, 3>(6, 6) = dR.transpose();
    } else {
        Fx.block<3, 3>(6, 6).setIdentity();
    }
    Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

    //Fi =
    //[0 0 0 0]
    //[I 0 0 0]
    //[0 I 0 0]
    //[0 0 I 0]
    //[0 0 0 I]

    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    //Qi =
    //[dt_2*acc_n    0    0    0]
    //[0    dt_2*gyr_n    0    0]
    //[0     0      dt*acc_w   0]
    //[0     0       0  dt*gyo_w]
    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt_2 * acc_noise_;
    Qi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt_2 * gyro_noise_;
    Qi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * dt * acc_bias_noise_;
    Qi.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * dt * gyro_bias_noise_;

    // P = Fx * P * Fxt + Fi * Q * Fit
    state_ptr_->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();
}

void ESKF::update(GNSSDataPtr gnss_data_ptr)
{
    //std::cout<<"[ ESKF ] GNSS and IMU timestamp: "<<gnss_data_ptr->timestamp<<", "<<state_ptr_->timestamp<<", "<<gnss_data_ptr->timestamp - state_ptr_->timestamp<<std::endl;
    double dt = gnss_data_ptr->timestamp - state_ptr_->timestamp;
    if(abs(dt) > 0.01) {
        std::cout<<"[ ESKF ] gnss - state time gap is too large " << dt <<std::endl;
        // return;
    }
    
    Eigen::Vector3d p_G_GNSS;
    convert_lla_to_enu(init_lla_, gnss_data_ptr->lla, &p_G_GNSS);
    Eigen::Vector3d &p_G_I = state_ptr_->p_G_I;
    Eigen::Matrix3d &R_G_I = state_ptr_->R_G_I;

    Eigen::Vector3d residual = p_G_GNSS - (p_G_I + R_G_I * p_I_GNSS_);
    Eigen::Matrix<double, 3, 15> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, 6) = - R_G_I * skew_matrix(p_I_GNSS_);
    Eigen::Matrix3d &V = gnss_data_ptr->cov;

    // ESKF
    // K = P * Ht * ( H * P * Ht + V).inverse();
    // dx = K * (y - h(x))
    // P = (I - KH) * P
    const Eigen::MatrixXd& P = state_ptr_->cov;
    const Eigen::MatrixXd  K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
    const Eigen::VectorXd delta_x = K * residual;

    // update state
    state_ptr_->p_G_I     += delta_x.block<3, 1>(0, 0);
    state_ptr_->v_G_I     += delta_x.block<3, 1>(3, 0);
    state_ptr_->acc_bias  += delta_x.block<3, 1>(9, 0);
    state_ptr_->gyro_bias += delta_x.block<3, 1>(12, 0);
    if (delta_x.block<3, 1>(6, 0).norm() > DBL_EPSILON) {
        state_ptr_->R_G_I *= v_expmap(delta_x.block<3, 1>(6, 0));
    }

    // update covarance.
    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    state_ptr_->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
}

bool ESKF::process_GNSS_Data(GNSSDataPtr gnss_data_ptr)
{
    if(!initialized_)
    {
        if(imu_buffer_.size() < IMU_Buffer_Size){
            std::cout<<"[ ESKF ] Wait. Insufficient IMU data."<<std::endl;
            return false;
        }
        last_imu_ptr_ = imu_buffer_.back();
        if(std::abs(last_imu_ptr_->timestamp - gnss_data_ptr->timestamp) > 0.2)
        {
            std::cout<<"[ ESKF ] GNSS and IMU are not sychonized."<<std::endl;
            return false;
        }
        if(!initialize()) return false;
        init_lla_ = gnss_data_ptr->lla;
        initialized_ = true;
    }

    update(gnss_data_ptr);
    return true;
}

bool ESKF::initialize(void)
{
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (auto imu_data : imu_buffer_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();
    std::cout<<"[ ESKF ] Mean acc: "<<mean_acc[0]<<" "<<mean_acc[1]<<" "<<mean_acc[2]<<std::endl;

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (auto imu_data : imu_buffer_) sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();
    if (std_acc.maxCoeff() > IMU_Std) {
        std::cout<<"[ ESKF ] Big acc std: "<<std_acc[0]<<" "<<std_acc[1]<<" "<<std_acc[2]<<std::endl;
        return false;
    }

    // z-axis
    const Eigen::Vector3d &z_axis = mean_acc.normalized();

    // x-axis
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d R_I_G;
    R_I_G.block<3, 1>(0, 0) = x_axis;
    R_I_G.block<3, 1>(0, 1) = y_axis;
    R_I_G.block<3, 1>(0, 2) = z_axis;

    state_ptr_->R_G_I = R_I_G.transpose();
    state_ptr_->timestamp = last_imu_ptr_->timestamp;
    //state_ptr_->imu_data_ptr = last_imu_ptr_;
    state_ptr_->p_G_I.setZero();
    state_ptr_->v_G_I.setZero();
    state_ptr_->angular.setZero();
    state_ptr_->acc_bias.setZero();
    state_ptr_->gyro_bias.setZero();
    state_ptr_->cov.setZero();
    state_ptr_->cov.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity();
    state_ptr_->cov.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity();
    state_ptr_->cov.block<2, 2>(6, 6) = 100. * D_R * D_R * Eigen::Matrix2d::Identity();
    state_ptr_->cov(8, 8) = 10000. * D_R * D_R;
    state_ptr_->cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();
    state_ptr_->cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();

    return true;
}
