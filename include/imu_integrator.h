#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

namespace radar_graph_slam {

class ImuIntegrator {
public:
    ImuIntegrator();
    void add_imu_message(const sensor_msgs::ImuConstPtr& imu_msg);
    Eigen::Isometry3d integrate(const ros::Time& from_time, const ros::Time& to_time, Eigen::Vector3d& velocity);
    void reset();

private:
    std::vector<sensor_msgs::ImuConstPtr> imu_buffer;
    Eigen::Vector3d gravity;
};

}
