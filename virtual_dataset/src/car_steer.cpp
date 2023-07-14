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

#include "cubicSpline.h"
#include "PID.h"
// MADE by TheConstruct , contact duckfrots@theconstructsim.com for any doubts or questions
// Or leave a request in the public git that contains this code.

namespace gazebo
{
  class CarSteer : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CarSteer::OnUpdate, this));
      
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
            boost::bind(&CarSteer::OnRosMsg_Pos, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&CarSteer::QueueThread, this));

      ROS_WARN("Loaded CarSteer Plugin with parent...%s, only X Axis Freq Supported in this V-1.0", this->model->GetName().c_str());
      
      //-------------------------------------------------------------------------
      this->model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
      this->model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
      this->pid_v = new PID(system_speed*3.0, system_speed*0.3, -0.3*system_speed, 0.01); 
      this->pid_pitch = new PID(0.0, 0.785, -0.785, 0.01);
      this->sp3 = new spline3D_interpolated(this->p_x, this->p_y, this->p_z, 0.1);
      this->pid_v->set_param(3.0*system_speed, 0.0, 0.005);
      this->model->SetGravityMode(false);
      std::cout << this->sp3->r_x.size()<<std::endl;
      this->Q_lqr << 50/system_speed, 0, 0, 0,
               0, 100/system_speed, 0, 0,
               0, 0, 20, 0,
               0, 0, 0, 6;
      double x = this->model->WorldPose().X();
      double y = this->model->WorldPose().Y();
      double z = this->model->WorldPose().Z();
      double yaw = this->model->WorldPose().Yaw();
      double pitch = this->model->WorldPose().Pitch();
      double roll = this->model->WorldPose().Roll();
      this->model->SetWorldPose(ignition::math::Pose3d(this->sp3->r_x[0], this->sp3->r_y[0], this->sp3->r_z[0], roll, this->sp3->r_pitch[0], this->sp3->r_yaw[0]));
      ofstream ofs;
      ofs.open("/home/greg/catkin_ws/src/test_mover/traj.txt", ios::out);
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
      // std::cout<<yaw<<std::endl;
      
      if (this->ind < this->sp3->r_x.size()-1){
        double* id = this->sp3->calc_nearest_index(x, y, z, yaw, this->ind);
        this->record_trajectory();
        this->ind = id[0];
        double v = this->model->WorldLinearVel().Length();
        double ideal = 3.0*(1.1 - min(4.0, floor(abs(this->sp3->r_k[this->ind]))) / 4)*system_speed;
        double e_pitch = this->sp3->pi_2_pi(this->sp3->r_pitch[this->ind] - pitch);
        this->pid_pitch->set_ideal(0);
        double angle = this->cal_angle(sp3, this->ind, id[1], v);
        this->angle_p = this->pid_pitch->calculate_output(-e_pitch);
        if (abs(this->last_angle - angle) > 0.174){
            if(this->last_angle < angle){
                angle = this->last_angle + 0.174;
            }
            else{angle = this->last_angle - 0.174;}
        }

        this->pid_v->set_ideal(max(0.1*system_speed, ideal));
        double accel = this->pid_v->calculate_output(v);
        
        double next_speed = max(0.1*system_speed, v+accel);
        this->model->SetLinearVel(ignition::math::Vector3d(next_speed* cosf32(yaw) * abs(cosf32(pitch)), next_speed* sinf32(yaw)*abs(cosf32(pitch)), -next_speed * sinf32(pitch)));
        // std::cout<<this->pe<<" "<<this->pth_e<<std::endl;

        if (this->sp3->r_k[this->ind]>3.0){
          double sp_angle = next_speed*angle*20/0.785;
          if (sp_angle>=3.49){sp_angle = 3.49;}
          else if(sp_angle<=-3.49){sp_angle = -3.49;}
          this->model->SetAngularVel(ignition::math::Vector3d(-next_speed/0.1*tanf32(this->angle_p)*sinf32(yaw), next_speed/0.1*tanf32(this->angle_p)*cosf32(yaw), sp_angle));
        }
        else{
          this->model->SetAngularVel(ignition::math::Vector3d(-next_speed/0.1*tanf32(this->angle_p)*sinf32(yaw), next_speed/0.1*tanf32(this->angle_p)*cosf32(yaw), next_speed/0.1*tanf32(angle))); //1.0 0.0
        }
        this->last_angle = angle;
        std::cout<<this->ind<<"/"<<this->sp3->r_x.size()<<" "<<id[1]<<std::endl;
        //this->model->SetAngularVel(ignition::math::Vector3d(0,0, (v*angle / 0.785)*1.34*times));

        // std::cout <<this->sp3->r_pitch[this->ind]<<" "<<pitch<<" "<<e_pitch<<" "<<" "<<this->angle_p<<std::endl;
        // std::cout<<v+accel<<" "<<angle<<" "<<this->ind<<std::endl;
      }
      else{
        this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
        this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
        std::cout<<"end"<<std::endl;
      }
    }

    std::vector<double> p_z = {1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3};
    std::vector<double> p_x = {0.0,-1.0,-3.0,-7.0,-9.0,-3.0, 0.0, 0.0, 2.0, 4.0, 6.0, 3.0, 0.0, 0.0, 4.0, 8.0, 7.0, 6.0, 6.0, 2.0, 0.0};
    std::vector<double> p_y = {0.0, 2.0, 3.0, 3.0, 1.0, 0.0,-2.0,-5.0,-7.0,-8.0,-6.0,-5.0,-2.0, 1.5, 2.0, 4.0, 7.0, 5.0, 3.0, 3.0, 0.0};
    spline3D_interpolated* sp3;
    PID* pid_v;
    PID* pid_pitch;
    int ind = 0;
    int end = 0;
    double angle_p = 0.0;
    double pe = 0.0;
    double pth_e = 0.0;
    double ppe = 0.0;
    double ppth_e = 0.0;
    int count = 0;
    double system_speed = 0.2;
    double last_angle = 0.0;

    Eigen::Matrix4d Q_lqr;
    Eigen::Matrix4d Q_lqr_p;
    double R_p = 1.0;
    double R_1 = 20*system_speed;

    int last_ind = 0;

    double calculate_yaw(double startx, double starty, double endx, double endy){
      double  vx = endx - startx;
      double  vy = endy - starty;
      return atan2(vy, vx);
    }

    double cal_angle(spline3D_interpolated* sp, int ind, double e, double v){
      double k = sp->r_k[ind];
      double th_e = sp->pi_2_pi(this->model->WorldPose().Yaw() - sp->r_yaw[ind]);

      Eigen::Matrix4d A;
      A << 1.0, 0.001, 0, 0,
           0, 0, v, 0,
           0, 0, 1.0, 0.001,
           0, 0, 0, 0;
      
      Eigen::Vector4d B;
      B << 0,
           0,
           0,
           v/0.1;

      Eigen::Matrix<double, 1, 4> K_lqr = this->dlqr(A, B,  this->Q_lqr, this->R_1);

      Eigen::Vector4d x;
      x << e,
           (e- this->pe) / 0.001,
           th_e,
           (th_e - this->pth_e) / 0.001;
      double ff= atan2(k*0.1, 1);
      // double ff= 0.0;
      double fb = sp->pi_2_pi((-K_lqr*x)(0, 0));

      double delta = fb + ff;
      if (delta > 0.785){delta = 0.785;}
      else if (delta < -0.785){delta = -0.785;}
      this->pe = e;
      this->pth_e = th_e;
      return delta;

    }

     double cal_angle_diff(spline3D_interpolated* sp, int ind, double e, double v){
      double k = sp->r_k[ind];
      double th_e = sp->pi_2_pi(this->model->WorldPose().Yaw() - sp->r_yaw[ind]);

      Eigen::Matrix4d A;
      A << 1.0, 0.001, 0, 0,
           0, 0, v, 0,
           0, 0, 1.0, 0.001,
           0, 0, 0, 0;
      
      Eigen::Vector4d B;
      B << 0,
           0,
           0,
           1;

      Eigen::Matrix<double, 1, 4> K_lqr = this->dlqr(A, B,  this->Q_lqr, this->R_1);

      Eigen::Vector4d x;
      x << e,
           (e- this->pe) / 0.001,
           th_e,
           (th_e - this->pth_e) / 0.001;
      double ff= atan2(k*0.1, 1);
      // double ff= 0.0;
      double fb = sp->pi_2_pi((-K_lqr*x)(0, 0));

      double delta = fb + ff;
      if (delta > 3.4){delta = 3.4;}
      else if (delta < -3.4){delta = -3.4;}
      this->pe = e;
      this->pth_e = th_e;
      return delta;

    }

    Eigen::Matrix4d solve_dare(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, double R){
      Eigen::Matrix4d x = Q;
      Eigen::Matrix4d x_next = Q;
      int max_iter = 50;
      double eps = 0.01;

      for(int i = 0; i<max_iter; i++){
        x_next = A.transpose()*x*A-A.transpose()*x*B*B.transpose()*x*A/(R+B.transpose()*x*B) + Q;
        if ((x_next - x).array().abs().maxCoeff() < eps){break;}
        x = x_next;
      }
      return x_next;
    }

    Eigen::Matrix<double, 1, 4> dlqr(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, double R){
      Eigen::Matrix4d X = this->solve_dare(A, B, Q, R);

      Eigen::Matrix<double, 1, 4> K_lqr = 1/(B.transpose()*X*B + R)*(B.transpose()*X*A);

      return K_lqr;
    }

    void record_trajectory(){
      double x = this->model->WorldPose().X();
      double y = this->model->WorldPose().Y();
      double z = this->model->WorldPose().Z();
      double yaw = this->model->WorldPose().Yaw();
      double pitch = this->model->WorldPose().Pitch();
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
      Eigen::Vector3d Z = {0, 0, 1};
      Eigen::Matrix3d R;
      R << 0, 1, 0,
          -1, 0, 0,
           0, 0, 1;
      Eigen::Vector3d v = {x, y, z};
      Eigen::Vector3d result = R*v;
      result = result + 0.1 * Y * P * Z; 
      ignition::math::v6::Quaternion<double> Q(cosf32(-M_PI / 4), 0.0, 0.0, sinf32(-M_PI / 4));
      q = Q*q;
       
      ofstream ofs;
      ofs.open("/home/greg/catkin_ws/src/test_mover/traj.txt", ios::app|ios::out);
      ofs <<timestamp<<" "<<result[0]<<" "<<result[1]<<" "<<result[2]<<" "<<q.X()<<" "<<q.Y()<<" "<<q.Z()<<" "<<q.W()<< endl;
      ofs.close();
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
  GZ_REGISTER_MODEL_PLUGIN(CarSteer);
}