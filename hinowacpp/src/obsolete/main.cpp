/*
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
*/
//testing main for hinowacpp functionality
#include <ros/ros.h>
#include <hinowacpp/can_interface.h>
#include <hinowacpp/joint.h>

int main(int argc, char** argv) {
	//ros setup
    	ros::init(argc, argv, "test");
    	ros::NodeHandle nh;
    	ros::AsyncSpinner spinner(1);
    	spinner.start();
	
	//hinowacpp::CAN can(0, true); // (int id, bool virtual)

	//uint32_t data [] = {0x1A,0x2A,0x3A,0x4A,0x5A,0x6A,0x7A,0x8A};
	//uint8_t* written_data = can.writeData(0x101, data);
//	int position;
	
//	hinowacpp::Joint joint_1(0x107, 0x101);
	//joint_1.setMotorId(0x107);
	//joint_1.setActuatorType(ACTUATOR_TYPE_MOTOR);
while(ros::ok())
{
        //double joint_position = joint_1.readAngle();
        //printf("joint position: %lf\n", joint_position);
	//joint_1.actuate(5000, 1);
	//uint8_t* written_data = can.writeData(0x101, data);
	//int read_data = can.readData(0x100, position);
}
    	//ros::spin();
	ros::waitForShutdown();
    
    	return 0;
}

