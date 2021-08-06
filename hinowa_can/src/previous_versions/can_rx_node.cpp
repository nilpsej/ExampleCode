#include <ros/ros.h>
#include <hinowa_can/can_bcm.h>
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "can_rx");
	ros::NodeHandle nh;
	bool _virt;
	nh.getParam("/CAN/virtual", _virt);

	printf("=============================CAN START-UP STATUS================================\n");
	if(_virt){
		system("sudo ip link add dev vcan0 type vcan");
		system("sudo ip link set up vcan0");
	}
	else system("sudo ip link set can0 up type can bitrate 250000");
	hinowa_can::can_bcm can_bcm(0, _virt, nh);
	int running;
	ros::Rate r(125);
	while(ros::ok()){
		running = can_bcm.run();
		can_bcm.publish();

		//rx_queue.callAvailable(ros::WallDuration(0.003)); //determine best value for this
		ros::spinOnce();
		r.sleep();
	}
	

	ros::shutdown();
	return 0;
}


