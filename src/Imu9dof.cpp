#include <iostream>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/physics/World.hh>
#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif
#include <ignition/math/Rand.hh>

#include "Imu9dof.h"

GZ_REGISTER_SENSOR_PLUGIN(gazebo::Imu9dofSensor)

gazebo::Imu9dofSensor::Imu9dofSensor(): SensorPlugin()
{
    accelerometer_data = ignition::math::Vector3d(0, 0, 0);
    gyroscope_data = ignition::math::Vector3d(0, 0, 0);
    orientation = ignition::math::Quaterniond(1,0,0,0);
    accel_bias = {0.0, 0.0, 0.0};
    gyro_bias = {0.0, 0.0, 0.0};
    sensor=NULL;
}

gazebo::Imu9dofSensor::~Imu9dofSensor()
{
    if (connection.get())
    {
        connection.reset();
        connection = gazebo::event::ConnectionPtr();
    }
    // close node, kill process
    node->shutdown();
}

// father is sensor_, configuration is sdf_
void gazebo::Imu9dofSensor::Load(gazebo::sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
    // get sensor and sdf here
    sdf=sdf_;
    sensor=dynamic_cast<gazebo::sensors::ImuSensor*>(sensor_.get());

    if(sensor==NULL)
    {
        ROS_FATAL("Error: Sensor pointer is NULL!");
        return;
    }

    bool initial_orientation_as_reference = false;
    if (!sdf->HasElement("initialOrientationAsReference"))
    {
        ROS_INFO("<initialOrientationAsReference> is unset, using default value of false "
                "to comply with REP 145 (world as orientation reference)");
    }
    else
    {
        initial_orientation_as_reference = sdf->Get<bool>("initialOrientationAsReference");
    }

    if (initial_orientation_as_reference)
    {
        ROS_WARN("<initialOrientationAsReference> set to true, this behavior is deprecated "
                "as it does not comply with REP 145.");
    }
    else
    {
        // This complies with REP 145
        sensor->SetWorldToReferenceOrientation(ignition::math::Quaterniond::Identity);
    }

    sensor->SetActive(true);

    if(!LoadParameters())
    {
        ROS_FATAL("Error Loading Parameters!");
        return;
    }

    if (!ros::isInitialized()) //check if ros is initialized properly
    {
        ROS_FATAL("ROS has not been initialized!");
        return;
    }

    node = new ros::NodeHandle(this->robot_namespace);

    imu_data_publisher = node->advertise<sensor_msgs::Imu>(topic_name,1);

    connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&Imu9dofSensor::UpdateChild, this, _1));

    last_time = sensor->LastUpdateTime();
}

void gazebo::Imu9dofSensor::UpdateChild(const gazebo::common::UpdateInfo &/*_info*/)
{
    #ifdef ENABLE_PROFILER
    IGN_PROFILE("Imu9dofSensor::UpdateChild");
    #endif
    common::Time current_time = sensor->LastUpdateTime();

    if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) //update rate check
        return;

    if(imu_data_publisher.getNumSubscribers() > 0)
    {
    #ifdef ENABLE_PROFILER
        IGN_PROFILE_BEGIN("fill ROS message");
    #endif
        double sqrt_dt = std::sqrt(dt);

        orientation = offset.Rot()*sensor->Orientation(); //applying offsets to the orientation measurement
        accelerometer_data = sensor->LinearAcceleration();
        gyroscope_data = sensor->AngularVelocity();

        //Guassian noise is applied to all measurements
        imu_msg.orientation.x = orientation.X() + GuassianKernel(0,gaussian_noise);
        imu_msg.orientation.y = orientation.Y() + GuassianKernel(0,gaussian_noise);
        imu_msg.orientation.z = orientation.Z() + GuassianKernel(0,gaussian_noise);
        imu_msg.orientation.w = orientation.W() + GuassianKernel(0,gaussian_noise);

        accel_bias[0] += GuassianKernel(0, random_walk_accel) * sqrt_dt;
        accel_bias[1] += GuassianKernel(0, random_walk_accel) * sqrt_dt;
        accel_bias[2] += GuassianKernel(0, random_walk_accel) * sqrt_dt;
        gyro_bias[0] += GuassianKernel(0, random_walk_gyro) * sqrt_dt;
        gyro_bias[1] += GuassianKernel(0, random_walk_gyro) * sqrt_dt;
        gyro_bias[2] += GuassianKernel(0, random_walk_gyro) * sqrt_dt;

        imu_msg.linear_acceleration.x = accelerometer_data.X() + GuassianKernel(0,noise_density_accel) / sqrt_dt + accel_bias[0];
        imu_msg.linear_acceleration.y = accelerometer_data.Y() + GuassianKernel(0,noise_density_accel) / sqrt_dt + accel_bias[1];
        imu_msg.linear_acceleration.z = accelerometer_data.Z() + GuassianKernel(0,noise_density_accel) / sqrt_dt + accel_bias[2];

        imu_msg.angular_velocity.x = gyroscope_data.X() + GuassianKernel(0,noise_density_gyro) / sqrt_dt + gyro_bias[0];
        imu_msg.angular_velocity.y = gyroscope_data.Y() + GuassianKernel(0,noise_density_gyro) / sqrt_dt + gyro_bias[1];
        imu_msg.angular_velocity.z = gyroscope_data.Z() + GuassianKernel(0,noise_density_gyro) / sqrt_dt + gyro_bias[2];
        //covariance is related to the Gaussian noise
        // double gn2 = gaussian_noise*gaussian_noise;
        // imu_msg.orientation_covariance[0] = 0.0;
        // imu_msg.orientation_covariance[4] = 0.0;
        // imu_msg.orientation_covariance[8] = 0.0;
        // imu_msg.angular_velocity_covariance[0] = 0.0;
        // imu_msg.angular_velocity_covariance[4] = 0.0;
        // imu_msg.angular_velocity_covariance[8] = 0.0;
        // imu_msg.linear_acceleration_covariance[0] = 0.0;
        // imu_msg.linear_acceleration_covariance[4] = 0.0;
        // imu_msg.linear_acceleration_covariance[8] = 0.0;

        //preparing message header
        imu_msg.header.frame_id = body_name;
        imu_msg.header.stamp.sec = current_time.sec;
        imu_msg.header.stamp.nsec = current_time.nsec;
    #ifdef ENABLE_PROFILER
        IGN_PROFILE_END();
        //publishing data
        IGN_PROFILE_BEGIN("publish");
    #endif
        imu_data_publisher.publish(imu_msg);
    #ifdef ENABLE_PROFILER
        IGN_PROFILE_END();
    #endif
        ros::spinOnce();
    }

    last_time = current_time;
}

// gaussian noise generator
double gazebo::Imu9dofSensor::GuassianKernel(double mu, double sigma)
{
    // generation of two normalized uniform random variables
    double U1 = ignition::math::Rand::DblUniform();
    double U2 = ignition::math::Rand::DblUniform();

    // using Box-Muller transform to obtain a varaible with a standard normal distribution
    double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

    // scaling
    Z0 = sigma * Z0 + mu;
    return Z0;
}

bool gazebo::Imu9dofSensor::LoadParameters()
{
    //loading parameters from the sdf file

    //NAMESPACE
    if (sdf->HasElement("robotNamespace"))
    {
        robot_namespace =  sdf->Get<std::string>("robotNamespace") +"/";
        ROS_INFO_STREAM("<robotNamespace> set to: "<<robot_namespace);
    }
    else
    {
        std::string scoped_name = sensor->ParentName();
        std::size_t it = scoped_name.find("::");

        robot_namespace = "/" +scoped_name.substr(0,it)+"/";
        ROS_WARN_STREAM("missing <robotNamespace>, set to default: " << robot_namespace);
    }

    //TOPIC
    if (sdf->HasElement("topicName"))
    {
        topic_name =  sdf->Get<std::string>("topicName");
        ROS_INFO_STREAM("<topicName> set to: "<<topic_name);
    }
    else
    {
        topic_name = "imu_data";
        ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
    }

    //BODY NAME
    if (sdf->HasElement("frameName"))
    {
        body_name =  sdf->Get<std::string>("frameName");
        ROS_INFO_STREAM("<frameName> set to: "<<body_name);
    }
    else
    {
        ROS_FATAL("missing <frameName>, cannot proceed");
        return false;
    }

    //UPDATE RATE
    if (sdf->HasElement("updateRateHZ"))
    {
        update_rate =  sdf->Get<double>("updateRateHZ");
        dt = 1 / update_rate;
        ROS_INFO_STREAM("<updateRateHZ> set to: " << update_rate);
    }
    else
    {
        update_rate = 1.0;
        dt = 1 / update_rate;
        ROS_WARN_STREAM("missing <updateRateHZ>, set to default: " << update_rate);
    }

    //NOISE for orientation
    if (sdf->HasElement("gaussianNoise"))
    {
        gaussian_noise =  sdf->Get<double>("gaussianNoise");
        ROS_INFO_STREAM("<gaussianNoise> set to: " << gaussian_noise);
    }
    else
    {
        gaussian_noise = 0.0;
        ROS_WARN_STREAM("missing <gaussianNoise>, set to default: " << gaussian_noise);
    }

    //NOISE for noise density
    if (sdf->HasElement("NoiseDensityAccel"))
    {
        noise_density_accel =  sdf->Get<double>("NoiseDensityAccel");
        ROS_INFO_STREAM("<NoiseDensityAccel> set to: " << noise_density_accel);
    }
    else
    {
        noise_density_accel = 0.0;
        ROS_WARN_STREAM("missing <NoiseDensityAccel>, set to default: " << noise_density_accel);
    }

    if (sdf->HasElement("NoiseDensityGyro"))
    {
        noise_density_gyro =  sdf->Get<double>("NoiseDensityGyro");
        ROS_INFO_STREAM("<NoiseDensityGyro> set to: " << noise_density_gyro);
    }
    else
    {
        noise_density_gyro = 0.0;
        ROS_WARN_STREAM("missing <NoiseDensityGyro>, set to default: " << noise_density_gyro);
    }

    // NOISE for random walk
    if (sdf->HasElement("RandomWalkAccel"))
    {
        random_walk_accel =  sdf->Get<double>("RandomWalkAccel");
        ROS_INFO_STREAM("<RandomWalkAccel> set to: " << random_walk_accel);
    }
    else
    {
        random_walk_accel = 0.0;
        ROS_WARN_STREAM("missing <RandomWalkAccel>, set to default: " << random_walk_accel);
    }

    if (sdf->HasElement("RandomWalkGyro"))
    {
        random_walk_gyro =  sdf->Get<double>("RandomWalkGyro");
        ROS_INFO_STREAM("<RandomWalkGyro> set to: " << random_walk_gyro);
    }
    else
    {
        random_walk_gyro = 0.0;
        ROS_WARN_STREAM("missing <RandomWalkGyro>, set to default: " << random_walk_gyro);
    }


    //POSITION OFFSET, UNUSED
    if (sdf->HasElement("xyzOffset"))
    {
        offset.Pos() =  sdf->Get<ignition::math::Vector3d>("xyzOffset");
        ROS_INFO_STREAM("<xyzOffset> set to: " << offset.Pos()[0] << ' ' << offset.Pos()[1] << ' ' << offset.Pos()[2]);
    }
    else
    {
        offset.Pos() = ignition::math::Vector3d(0, 0, 0);
        ROS_WARN_STREAM("missing <xyzOffset>, set to default: " << offset.Pos()[0] << ' ' << offset.Pos()[1] << ' ' << offset.Pos()[2]);
    }

    //ORIENTATION OFFSET
    if (sdf->HasElement("rpyOffset"))
    {
        offset.Rot() = ignition::math::Quaterniond(sdf->Get<ignition::math::Vector3d>("rpyOffset"));
        ROS_INFO_STREAM("<rpyOffset> set to: " << offset.Rot().Roll() << ' ' << offset.Rot().Pitch() << ' ' << offset.Rot().Yaw());
    }
    else
    {
        offset.Rot() = ignition::math::Quaterniond::Identity;
        ROS_WARN_STREAM("missing <rpyOffset>, set to default: " << offset.Rot().Roll() << ' ' << offset.Rot().Pitch() << ' ' << offset.Rot().Yaw());
    }

    return true;
}

