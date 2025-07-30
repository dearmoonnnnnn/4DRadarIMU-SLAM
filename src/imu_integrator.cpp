#include <imu_integrator.h>

namespace radar_graph_slam {

ImuIntegrator::ImuIntegrator() {
    gravity = Eigen::Vector3d(0, 0, -9.80665);
}

void ImuIntegrator::add_imu_message(const sensor_msgs::ImuConstPtr& imu_msg) {
    imu_buffer.push_back(imu_msg);
}

Eigen::Isometry3d ImuIntegrator::integrate(const ros::Time& from_time, const ros::Time& to_time, Eigen::Vector3d& velocity) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();

    auto it = std::lower_bound(imu_buffer.begin(), imu_buffer.end(), from_time, 
        [](const sensor_msgs::ImuConstPtr& msg, const ros::Time& time) {
        return msg->header.stamp < time;
    });

    if (it == imu_buffer.begin()) {
        // If the first message is after from_time, we can't integrate
        if (it != imu_buffer.end() && (*it)->header.stamp > from_time) {
            it++;
        }
    }
    
    if (it == imu_buffer.end()) {
        return pose;
    }

    ros::Time last_time = from_time;

    for (; it != imu_buffer.end() && (*it)->header.stamp <= to_time; ++it) {
        const auto& imu_msg = *it;
        double dt = (imu_msg->header.stamp - last_time).toSec();
        if (dt <= 0) {
            continue;
        }

        Eigen::Vector3d acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
        Eigen::Vector3d gyro(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

        Eigen::Quaterniond delta_q(1, gyro.x() * dt / 2, gyro.y() * dt / 2, gyro.z() * dt / 2);
        delta_q.normalize();
        
        orientation = orientation * delta_q;
        
        Eigen::Vector3d acc_world = orientation * acc - gravity;
        
        position += velocity * dt + 0.5 * acc_world * dt * dt;
        velocity += acc_world * dt;

        last_time = imu_msg->header.stamp;
    }

    pose.linear() = orientation.toRotationMatrix();
    pose.translation() = position;

    // Clean up old imu messages
    imu_buffer.erase(imu_buffer.begin(), it);

    return pose;
}

void ImuIntegrator::reset() {
    imu_buffer.clear();
}

}
