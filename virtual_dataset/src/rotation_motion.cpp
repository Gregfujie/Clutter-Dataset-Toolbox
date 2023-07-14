#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <vector>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

// MADE by TheConstruct , contact duckfrots@theconstructsim.com for any doubts or questions
// Or leave a request in the public git that contains this code.

namespace gazebo
{
  class RotationMotion : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&RotationMotion::OnUpdate, this));
        
        this->old_secs =ros::Time::now().toSec();
        
        // Create a topic name
        std::string plannar_pos_topicName = "/cmd_vel";

        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "plannar_rosnode",
                ros::init_options::NoSigintHandler);
        }
            
        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("plannar_rosnode"));
        
        // Plannar Pose
        ros::SubscribeOptions so =
            ros::SubscribeOptions::create<geometry_msgs::Twist>(
                plannar_pos_topicName,
                1,
                boost::bind(&RotationMotion::OnRosMsg_Pos, this, _1),
                ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);
        
        // Spin up the queue helper thread.
        this->rosQueueThread =
            std::thread(std::bind(&RotationMotion::QueueThread, this));

        ROS_WARN("Loaded RotationMotion Plugin with parent...%s, only X Axis Freq Supported in this V-1.0", this->model->GetName().c_str());
        
        //-------------------------------------------------------------------------
        this->model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
        this->model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
        this->model->SetGravityMode(false);
        double x = this->model->WorldPose().X();
        double y = this->model->WorldPose().Y();
        double z = this->model->WorldPose().Z();
        double yaw = this->model->WorldPose().Yaw();
        double pitch = this->model->WorldPose().Pitch();
        double roll = this->model->WorldPose().Roll();

        // auto joints = this->model->GetLinks();
        // for (int i = 0; i< joints.size(); i++){
        //     std::cout<<joints[i]->GetName()<<std::endl;
        // }

        // physics::JointPtr joint = this->model->GetJoint("ouster_base_mount_joint");
        // std::cout<<this->model->GetLinks<<std::endl;
        // if (this->lidarLink){std::cout<<"success"<<std::endl;}     
        
        std::ofstream ofs;
        ofs.open("/home/greg/catkin_ws/src/test_mover/rot_traj.txt", std::ios::out);
        ofs.close();
        //--------------------------------------------------------
    }

    public: void OnUpdate()
    {
        double x = this->model->WorldPose().X();
        double y = this->model->WorldPose().Y();
        double z = this->model->WorldPose().Z();
        double yaw = this->model->WorldPose().Yaw();
        double pitch = this->model->WorldPose().Pitch();
        double roll = this->model->WorldPose().Roll();
        double state = this->angle / 6.2831;
        double speed = 0.2 * std::max((-4*pow((state - 0.5), 2)+1), 0.2);
        if (step == 0){          
            this->model->SetAngularVel(ignition::math::Vector3d(speed, 0, 0));     
            this->angle += speed*0.001;
            if (abs(this->angle - 6.2831) < 0.0005 || this->angle > 6.2831){
                step += 1;
                this->angle = 0;
                std::cout<<"step+1"<<std::endl;
            }
        }
        else if (step == 1){
            this->model->SetAngularVel(ignition::math::Vector3d(0, speed, 0));
            this->angle += speed*0.001;
            if (abs(this->angle - 6.2831) < 0.001 || this->angle > 6.2831){
                step += 1;
                this->angle = 0;
                std::cout<<"step+1"<<std::endl;
            }
        }
        else if (step == 2){
            this->model->SetAngularVel(ignition::math::Vector3d(0, 0, speed));
            this->angle += speed*0.001;
            if (abs(this->angle - 6.2831) < 0.001 || this->angle > 6.2831){
                step += 1;
                this->angle = 0;
                std::cout<<"step+1"<<std::endl;
            }
        }
        else{
            this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
            if (this->end == 0){
                std::cout<<"end"<<std::endl;
                end = 1;
            }
        }
        this->record_trajectory();
        // double v = this->model->WorldLinearVel().Length();
        // this->model->SetLinearVel(ignition::math::Vector3d(next_speed* cosf32(yaw) * abs(cosf32(pitch)), next_speed* sinf32(yaw)*abs(cosf32(pitch)), -next_speed * sinf32(pitch)));

        // this->model->SetAngularVel(ignition::math::Vector3d(-next_speed/0.1*tanf32(this->angle_p)*sinf32(yaw), next_speed/0.1*tanf32(this->angle_p)*cosf32(yaw), sp_angle));
    //   else{
    //     this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
    //     this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
    //     std::cout<<"end"<<std::endl;
    //   }
    }

    // std::vector<int> states = {0, 1, 2}; 
    //0 roll, 1 pitch, 2 yaw
    int step = 0;
    double angle = 0.0;
    int end = 0;
    physics::LinkPtr lidarLink;

    double calculate_yaw(double startx, double starty, double endx, double endy){
      double  vx = endx - startx;
      double  vy = endy - starty;
      return atan2(vy, vx);
    }

    void record_trajectory(){
        double x = this->model->WorldPose().X();
        double y = this->model->WorldPose().Y();
        double z = this->model->WorldPose().Z();
        double yaw = this->model->WorldPose().Yaw();
        double pitch = this->model->WorldPose().Pitch();
        double roll = this->model->WorldPose().Roll();
        double timestamp = this->model->GetWorld()->SimTime().Double();
        ignition::math::v6::Quaternion<double> q = this->model->WorldPose().Rot().Normalized();
        Eigen::Matrix3d P;
        P << cosf32(pitch), 0, sinf32(pitch),
            0, 1, 0,
            -sinf32(pitch), 0, cosf32(pitch);
        Eigen::Matrix3d Y;
        Y << cosf32(yaw), -sinf32(yaw), 0,
            sinf32(yaw), cosf32(yaw), 0,
            0, 0, 1;
        Eigen::Matrix3d R;
        R << 1, 0, 0,
             0, cosf32(roll), -sinf32(roll),
             0, sinf32(roll), cosf32(roll);
        Eigen::Vector3d Z = {0, 0, 1};
        Eigen::Matrix3d T;
        T << 0, 1, 0,
            -1, 0, 0,
            0, 0, 1;
        Eigen::Vector3d v = {x, y, z};
        Eigen::Vector3d result = T*v;
        result = result + 0.5 * Y * P * R * Z; 
        ignition::math::v6::Quaternion<double> Q(cosf32(-M_PI / 4), 0.0, 0.0, sinf32(-M_PI / 4));
        q = Q*q;
        
        std::ofstream ofs;
        ofs.open("/home/greg/catkin_ws/src/test_mover/rot_traj.txt", std::ios::app|std::ios::out);
        ofs <<timestamp<<" "<<result[0]<<" "<<result[1]<<" "<<result[2]<<" "<<q.X()<<" "<<q.Y()<<" "<<q.Z()<<" "<<q.W()<< std::endl;
        ofs.close();
    }

    void MoveModelsPlane(float linear_x_vel, float linear_y_vel, float linear_z_vel, float angular_x_vel, float angular_y_vel, float angular_z_vel)
    {

        std::string model_name = this->model->GetName();

        ROS_DEBUG("Moving model=%s",model_name.c_str());

        this->model->SetLinearVel(ignition::math::Vector3d(linear_x_vel, linear_y_vel, linear_z_vel));
        this->model->SetAngularVel(ignition::math::Vector3d(angular_x_vel, angular_y_vel, angular_z_vel));

        ROS_DEBUG("Moving model=%s....END",model_name.c_str());

    }
    
    public: void OnRosMsg_Pos(const geometry_msgs::TwistConstPtr &_msg)
    {
        this->MoveModelsPlane(_msg->linear.x, _msg->linear.y,_msg->linear.z, _msg->angular.x, _msg->angular.y, _msg->angular.z);
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Time Memory
    double old_secs;
    
    // Direction Value
    int direction = 1;
    // Frequency of earthquake
    double x_axis_pos = 1.0;
    // Magnitude of the Oscilations
    double y_axis_pos = 1.0;
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub2;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue2;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread2;

    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RotationMotion);
}