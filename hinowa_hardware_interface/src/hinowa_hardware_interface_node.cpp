#include <hinowa_hardware_interface/hinowa_hardware_interface.h>

int main(int argc, char** argv)
{
	//bool dynamicEncoders;
	ros::init(argc, argv, "hinowa_hardware_interface");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);

	//Stop the controller from interfering with mouse control.
	system("xinput set-prop \"PLAYSTATION(R)3 Controller\" \"Device Enabled\" 0");

	//struct stat buffer;
	//if(stat("/dev/input/js0", &buffer) != 0) system("sudo ds4drv --hidraw"); 

//--config ~/workspace/src/ps4_config/ds4drv.conf
//--emulate-xpad-wireless
	//Camera transmission setting.
	//system("sudo /usr/bin/jetson_clocks");//start cpu fan
	system("sudo ip link set eth0 mtu 8192"); //jumbo frames


	hinowa_joint_errors::JOINT_ERROR_SUBSCRIBER joint_error_subscriber = hinowa_joint_errors::JOINT_ERROR_SUBSCRIBER(nh);
	hinowa_joint_errors::JOINT_ERROR_PUBLISHER joint_error_publisher = hinowa_joint_errors::JOINT_ERROR_PUBLISHER(nh);
	hinowacpp::CAN_RX_SUBSCRIBER can_rx(nh);
	hinowacpp::CAN_TX_PUBLISHER can_tx(nh);
	hinowacpp::HINOWA goldlift = hinowacpp::HINOWA(can_rx, nh);
	hinowacpp::REMOTE ps3_remote = hinowacpp::REMOTE(nh);
	hinowacpp::LEVEL_SENSOR level_sensor = hinowacpp::LEVEL_SENSOR();
	hinowacpp::STATUS_SUBSCRIBER trajectory_status = hinowacpp::STATUS_SUBSCRIBER();
	camera_tf2::ARUCO_SUBSCRIBER aruco_sub = camera_tf2::ARUCO_SUBSCRIBER(nh);
	hinowacpp::encoder_filter filter = hinowacpp::encoder_filter();
	hinowa_parameters::hwi_publisher HWI_pub = hinowa_parameters::hwi_publisher(nh);
	hinowa_parameters::ctrl_loop_subscriber control_loop_sub = hinowa_parameters::ctrl_loop_subscriber(nh);

	spinner.start();
	hinowa_hardware_interface::hinowaHardwareInterface hinowa(nh, goldlift, ps3_remote, joint_error_subscriber, level_sensor, trajectory_status, aruco_sub, can_rx, can_tx, filter, HWI_pub, control_loop_sub, joint_error_publisher);
	ros::waitForShutdown();
	return 0;
}
