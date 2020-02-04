#ifndef ROS_CONTROL__HCR_HARDWARE_INTERFACE_H
#define ROS_CONTROL__HCR_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <hcr_hardware_interface/hcr_hardware.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>



using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;


namespace hcr_hardware_interface
{
    static const double POSITION_STEP_FACTOR = 10;
    static const double VELOCITY_STEP_FACTOR = 10;

    class HCRHardwareInterface: public hcr_hardware_interface::HCRHardware
    {
        public:
            HCRHardwareInterface(ros::NodeHandle& nh);
            ~HCRHardwareInterface();
            void init();
            void update(const ros::TimerEvent& e);
            void read();
            void write(ros::Duration elapsed_time);


            /*********************************************/
            void write_refVelM1(double refVelMotor);
            void write_refVelM2(double refVelMotor);

            double read_velM1();
            double read_velM2();

        protected:

            ros::NodeHandle nh_;
            ros::Timer non_realtime_loop_;
            ros::Duration control_period_;
            ros::Duration elapsed_time_;
            PositionJointInterface positionJointInterface;
            PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
            double loop_hz_;
            boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
            double p_error_, v_error_, e_error_;

            /*********************************************/
            ros::Subscriber _sub_velM1;
            ros::Subscriber _sub_velM2;

            ros::Publisher  _pub_refVelM1;
            ros::Publisher  _pub_refVelM2;

            double _velM1;
            double _velM2;

            void initializeSubscribers();
            void initializePublishers();

            void callback_velM1(const std_msgs::Float32& msg_vel);
            void callback_velM2(const std_msgs::Float32& msg_vel);

            std_msgs::Float32 _msg_refVel;

    };

}

#endif


