#ifndef ROS_CONTROL__HINOWA_HARDWARE_INTERFACE_H
#define ROS_CONTROL__HINOWA_HARDWARE_INTERFACE_H

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
#include <hinowa_hardware_interface/hinowa_hardware.h>
#include <hinowacpp/hinowa.h>
#include <hinowacpp/remote.h>
#include <hinowacpp/level_sensor.h>
#include <hinowacpp/status_subscriber.h>
#include <camera_tf2/aruco_subscriber.h>
#include <hinowacpp/can_rx_subscriber.h>
#include <hinowacpp/can_tx_publisher.h>
#include <hinowacpp/encoder_filter.h>
#include <hinowa_parameters/hwi_publisher.h>
#include <hinowa_parameters/ctrl_loop_subscriber.h>
#include <hinowa_joint_errors/joint_error_subscriber.h>
#include <hinowa_joint_errors/joint_error_publisher.h>
#include <sys/stat.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;


namespace hinowa_hardware_interface
{
    class hinowaHardwareInterface: public hinowa_hardware_interface::hinowaHardware
    {
        public:
            hinowaHardwareInterface(ros::NodeHandle& nh, hinowacpp::HINOWA& hinowa, hinowacpp::REMOTE& remote, hinowa_joint_errors::JOINT_ERROR_SUBSCRIBER& joint_error_subscriber, hinowacpp::LEVEL_SENSOR& level_sensor, hinowacpp::STATUS_SUBSCRIBER& trajectory_status, camera_tf2::ARUCO_SUBSCRIBER& aruco_sub, hinowacpp::CAN_RX_SUBSCRIBER& can_rx, hinowacpp::CAN_TX_PUBLISHER& can_tx, hinowacpp::encoder_filter& filter, hinowa_parameters::hwi_publisher& HWI_pub, hinowa_parameters::ctrl_loop_subscriber& control_loop_sub, hinowa_joint_errors::JOINT_ERROR_PUBLISHER& joint_error_publisher);
            ~hinowaHardwareInterface();
            void init();
            void update(const ros::TimerEvent& e);
            void read(ros::Duration elapsed_time);
            void write(ros::Duration elapsed_time);

        protected:
	    hinowa_joint_errors::JOINT_ERROR_SUBSCRIBER joint_error_subscriber_;
	    hinowa_joint_errors::JOINT_ERROR_PUBLISHER joint_error_publisher_;
	    hinowa_parameters::hwi_publisher HWI_pub_;
	    hinowa_parameters::ctrl_loop_subscriber control_loop_sub_;
	    camera_tf2::ARUCO_SUBSCRIBER aruco_sub_;
	    hinowacpp::STATUS_SUBSCRIBER trajectory_status_;
	    hinowacpp::REMOTE remote_;
	    hinowacpp::LEVEL_SENSOR level_sensor_;
            hinowacpp::HINOWA hinowa_;
	    hinowacpp::CAN_RX_SUBSCRIBER can_rx_;
	    hinowacpp::CAN_TX_PUBLISHER can_tx_;
	    hinowacpp::encoder_filter filter_;
            ros::NodeHandle nh_;
            ros::Timer non_realtime_loop_;
            ros::Duration control_period_;
            ros::Duration elapsed_time_;
            PositionJointInterface positionJointInterface;
            PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
            double loop_hz_;
            boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
            double p_error_, v_error_, e_error_;
	    std::vector<geometry_msgs::TransformStamped> aruco_anchor;
	    std::vector<geometry_msgs::TransformStamped> aruco_camera;
	    geometry_msgs::TransformStamped aruco0_camera;
	    std::tuple<std::vector<geometry_msgs::TransformStamped>, std::vector<geometry_msgs::TransformStamped>, int> aruco_sub_output;

    };

}

#endif
