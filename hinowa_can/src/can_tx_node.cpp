#include <ros/ros.h>
#include <hinowa_can/can_raw.h>
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "can_tx");
	ros::NodeHandle nh;
//	ros::CallbackQueue tx_queue;
//	nh.setCallbackQueue(&tx_queue);
	bool _virt;
	nh.getParam("/CAN/virtual", _virt);
	if(_virt){
		system("sudo modprobe vcan");
		system("sudo ip link add dev vcan0 type vcan");
		system("sudo ip link set up vcan0");
	}
	else system("sudo ip link set can0 up type can bitrate 250000");
	hinowa_can::can_raw can_raw(0, _virt, nh);
	ros::spin();
	return 0;

}
