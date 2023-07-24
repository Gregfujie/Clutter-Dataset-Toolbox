#ifndef GAZEBO_IMU_9dof_PLUGIN_H
#define GAZEBO_IMU_9dof_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>

namespace gazebo
{
namespace sensors
{
    class ImuSensor;
}
class Imu9dofSensor : public SensorPlugin
{
public:
    Imu9dofSensor();
    virtual ~Imu9dofSensor();
    virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

protected:
    virtual void UpdateChild(const gazebo::common::UpdateInfo &/*_info*/);

private:
    bool LoadParameters();
    double GuassianKernel(double mu, double sigma);
    
    ros::NodeHandle* node;
    ros::Publisher imu_data_publisher;
    sensor_msgs::Imu imu_msg;

    common::Time last_time;
    gazebo::event::ConnectionPtr connection;
    sensors::ImuSensor* sensor;
    sdf::ElementPtr sdf;
    ignition::math::Quaterniond orientation;
    ignition::math::Vector3d accelerometer_data;
    ignition::math::Vector3d gyroscope_data;

    //loaded parameters
    std::string robot_namespace;
    std::string topic_name;
    std::string body_name;
    double update_rate;
    double gaussian_noise;
    double noise_density_accel;
    double noise_density_gyro;
    double random_walk_accel;
    double random_walk_gyro;

    double dt;

    ignition::math::Vector3d accel_bias;
    ignition::math::Vector3d gyro_bias;
    ignition::math::Pose3d offset;
};
}

#endif GAZEBO_IMU_9dof_PLUGIN_H