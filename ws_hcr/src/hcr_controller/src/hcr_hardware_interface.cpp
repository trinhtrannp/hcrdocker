#include <sstream>
#include <hcr_hardware_interface/hcr_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using namespace hardware_interface;
using namespace hcr_hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
/* using joint_limits_interface::PositionJointSoftLimitsHandle; */
/* using joint_limits_interface::PositionJointSoftLimitsInterface; */
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;

namespace hcr_hardware_interface
{
    HCRHardwareInterface::HCRHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {

        initializeSubscribers();
        initializePublishers();

        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/hardware_interface/loop_hz", loop_hz_, 10.0);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &HCRHardwareInterface::update, this);


    }

    HCRHardwareInterface::~HCRHardwareInterface() {

    }

    void HCRHardwareInterface::init() {
        // Get joint names
        nh_.getParam("/hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {
          //
             // Create joint state interface
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            // Create position joint interface
            /* JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]); */
            /* positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle); */

            /* JointLimits limits; */
            /* SoftJointLimits softLimits; */
            /* getJointLimits(joint_names_[i], nh_, limits); */
            /* PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits); */
            /* position_joint_interface_.registerHandle(jointPositionHandle); */

            // Create velocity joint interface
            JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
            velocity_joint_interface_.registerHandle(jointVelocityHandle);

            JointLimits limits;
            SoftJointLimits softLimits;
            getJointLimits(joint_names_[i], nh_, limits);
            VelocityJointSoftLimitsHandle velocityJointSoftLimitsHandle(jointVelocityHandle, limits, softLimits);
            velocity_joint_limits_interface_.registerHandle(velocityJointSoftLimitsHandle);

            // Create effort joint interface
            /* JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]); */
            /* effort_joint_interface_.registerHandle(jointEffortHandle); */

        }

        registerInterface(&joint_state_interface_);

        /* registerInterface(&position_joint_interface_); */
        /* registerInterface(&positionJointSoftLimitsInterface); */

        registerInterface(&velocity_joint_interface_);
        registerInterface(&velocity_joint_limits_interface_);

        /* registerInterface(&effort_joint_interface_); */
    }

    void HCRHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void HCRHardwareInterface::read() {

        joint_position_[0] = read_velM1();
        joint_position_[1] = read_velM2();

        /* ROS_INFO("Get callback velM1: %f", joint_position_[0]); */
    }

    void HCRHardwareInterface::write(ros::Duration elapsed_time) {
        velocity_joint_limits_interface_.enforceLimits(elapsed_time);
        write_refVelM1(joint_velocity_command_[0]);
        write_refVelM2(joint_velocity_command_[1]);
    }


    /* HCR HW Interface functions                 */ 
    /**********************************************/
    void HCRHardwareInterface::initializeSubscribers()
    {
        ROS_INFO("Initializing Subscribers");
        _sub_velM1 = nh_.subscribe("hcr_velM1", 1, &HCRHardwareInterface::callback_velM1,this);  
        _sub_velM2 = nh_.subscribe("hcr_velM2", 1, &HCRHardwareInterface::callback_velM2,this);  
    }

    void HCRHardwareInterface::initializePublishers()
    {
        ROS_INFO("Initializing Publishers");
        _pub_refVelM1 = nh_.advertise<std_msgs::Float32>("hcr_refVelM1", 1, true); 
        _pub_refVelM2 = nh_.advertise<std_msgs::Float32>("hcr_refVelM2", 1, true); 
    }

    void HCRHardwareInterface::callback_velM1(const std_msgs::Float32& msg_vel) {

        _velM1 = (double)(msg_vel.data);
    }

    void HCRHardwareInterface::callback_velM2(const std_msgs::Float32& msg_vel) {

        _velM2 = (double)(msg_vel.data);
    }

    void HCRHardwareInterface::write_refVelM1(double refVelMotor) {

        _msg_refVel.data = refVelMotor; 
        _pub_refVelM1.publish(_msg_refVel); //output the square of the received value; 
    }

    void HCRHardwareInterface::write_refVelM2(double refVelMotor) {

        _msg_refVel.data = refVelMotor; 
        _pub_refVelM2.publish(_msg_refVel); //output the square of the received value; 
    }

    double HCRHardwareInterface::read_velM1() {
        
      return _velM1;
    }

    double HCRHardwareInterface::read_velM2() {
        
      return _velM2;
    }
}
