#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "../include/eskf.h"

class ESKF_Fusion
{
public:
    ESKF_Fusion(ros::NodeHandle nh){
        std::string imu_topic = "/imu_raw";
        std::string gps_topic = "/gps/fix";
        std::string odom_topic = "/nav/odom";
        std::string path_topic = "/nav/path";
        std::string save_path = "/home/work/";
        double acc_n, gyr_n, acc_w, gyr_w;
        double x, y, z;
        nh.param("imu_topic", imu_topic);
        nh.param("gps_topic", gps_topic);
        nh.param("acc_noise", acc_n, 1e-2);
        nh.param("gyr_noise", gyr_n, 1e-4);
        nh.param("acc_bias_noise", acc_w, 1e-6);
        nh.param("gyr_bias_noise", gyr_w, 1e-8);
        nh.param("p_I_GNSS_x", x, 0.);
        nh.param("p_I_GNSS_y", y, 0.);
        nh.param("p_I_GNSS_z", z, 0.);
        nh.param("odom_topic", odom_topic);
        nh.param("path_topic", path_topic);
        nh.param("save_path", save_path);
        const Eigen::Vector3d p_I_GNSS(x, y, z);
        eskf_ptr_ = std::make_shared<ESKF>(acc_n, gyr_n, acc_w, gyr_w, p_I_GNSS);

        // ROS sub & pub
        sub_imu_ = nh.subscribe(imu_topic, 10, &ESKF_Fusion::imu_callback, this);
        sub_gnss_ = nh.subscribe(gps_topic, 10, &ESKF_Fusion::gnss_callback, this);

        pub_odom_ = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
        pub_path_ = nh.advertise<nav_msgs::Path>(path_topic, 10);

        // log files
        file_gnss_.open(save_path + "gnss.csv");
        file_state_.open(save_path + "fused_state.csv");

        std::cout<<"[ ESKF ] Start."<<std::endl;
    }
    ~ESKF_Fusion()
    {
        file_gnss_.close();
        file_state_.close();
    }
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void gnss_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg);
    void publish_save_state(void);

private:
   ros::Subscriber sub_imu_;
   ros::Subscriber sub_gnss_;
   ros::Publisher pub_path_;
   ros::Publisher pub_odom_;
   nav_msgs::Path nav_path_;

   ESKFPtr eskf_ptr_;

   // log files
   std::ofstream file_gnss_;
   std::ofstream file_state_;
};

void ESKF_Fusion::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    IMUDataPtr imu_data_ptr = std::make_shared<IMUData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyro[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyro[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyro[2] = imu_msg->angular_velocity.z;
    if(!eskf_ptr_->process_IMU_Data(imu_data_ptr)) return;
}

void ESKF_Fusion::gnss_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg)
{
    if(gnss_msg->status.status < 0) // -1无法修复位置, 0未增强修复, 1使用卫星增强 2使用地面增强
    {
        std::cout<<"[ ESKF ] Bad GNSS data."<<std::endl;
        return;
    }
    GNSSDataPtr gnss_data_ptr = std::make_shared<GNSSData>();
    gnss_data_ptr->timestamp = gnss_msg->header.stamp.toSec();
    gnss_data_ptr->lla[0] = gnss_msg->latitude;
    gnss_data_ptr->lla[1] = gnss_msg->longitude;
    gnss_data_ptr->lla[2] = gnss_msg->altitude;
    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gnss_msg->position_covariance.data());
    if(!eskf_ptr_->process_GNSS_Data(gnss_data_ptr)) return;
    file_gnss_ << std::fixed << std::setprecision(15)
               << gnss_data_ptr->timestamp << ", "
               << gnss_data_ptr->lla[0] << ", "
               << gnss_data_ptr->lla[1] <<", "
               << gnss_data_ptr->lla[2] << std::endl;
    
    // gnss帧率低，所以只在gnss修正后再发布odom和path
    publish_save_state();
}


void ESKF_Fusion::publish_save_state(void)
{
    // publish the odometry
    std::string fixed_id = "global";
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = fixed_id;
    odom_msg.header.stamp = ros::Time::now();
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = eskf_ptr_->state_ptr_->R_G_I;
    T_wb.translation() = eskf_ptr_->state_ptr_->p_G_I;
    tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
    tf::vectorEigenToMsg(eskf_ptr_->state_ptr_->v_G_I, odom_msg.twist.twist.linear);
    tf::vectorEigenToMsg(eskf_ptr_->state_ptr_->angular, odom_msg.twist.twist.angular);
    Eigen::Matrix3d P_pp = eskf_ptr_->state_ptr_->cov.block<3, 3>(0, 0);// position covariance
    Eigen::Matrix3d P_po = eskf_ptr_->state_ptr_->cov.block<3, 3>(0, 6);// position rotation covariance
    Eigen::Matrix3d P_op = eskf_ptr_->state_ptr_->cov.block<3, 3>(6, 0);// rotation position covariance
    Eigen::Matrix3d P_oo = eskf_ptr_->state_ptr_->cov.block<3, 3>(6, 6);// rotation covariance
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
    P_imu_pose << P_pp, P_po, P_op, P_oo;
    for (int i = 0; i < 36; i++)
        odom_msg.pose.covariance[i] = P_imu_pose.data()[i];
    pub_odom_.publish(odom_msg);

    // publish the path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    nav_path_.header = pose_stamped.header;
    nav_path_.poses.push_back(pose_stamped);
    pub_path_.publish(nav_path_);

    // save state p q lla
    Eigen::Vector3d lla;
    convert_enu_to_lla(eskf_ptr_->init_lla_, eskf_ptr_->state_ptr_->p_G_I, &lla);
    Eigen::Quaterniond q_G_I(eskf_ptr_->state_ptr_->R_G_I);
    file_state_ << std::fixed << std::setprecision(15) << eskf_ptr_->state_ptr_->timestamp <<", "
                << eskf_ptr_->state_ptr_->p_G_I[0] << ", " << eskf_ptr_->state_ptr_->p_G_I[1] << ", " << eskf_ptr_->state_ptr_->p_G_I[2] << ", "
                << q_G_I.x() << ", " << q_G_I.y() << ", " << q_G_I.z() << ", " << q_G_I.w() << ", "
                << lla[0] << ", " << lla[1] << ", " << lla[2]
                << std::endl;
    std::cout << "bias of acc: "<<eskf_ptr_->state_ptr_->acc_bias.transpose()<<std::endl;
    std::cout << "bias of gyr: "<<eskf_ptr_->state_ptr_->gyro_bias.transpose()<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_gnss_eskf_node");
    ros::NodeHandle nh;
    ESKF_Fusion node(nh);
    ros::spin();
}
