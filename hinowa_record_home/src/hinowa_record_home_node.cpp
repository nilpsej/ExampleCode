#include <hinowacpp/can_rx_subscriber.h>
#include <ros/ros.h>

//TODO - have this spit out the hex received for each joint to the same file (ideally seperate vector) for when setting vcan generators
int main(int argc, char** argv)
{
	ros::init(argc, argv, "hinowa_record_home");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);

	hinowacpp::CAN_RX_SUBSCRIBER can_rx(nh);
	uint16_t encoder_IDs[5] = {0x181, 0x182, 0x183, 0x184, 0x185};
	RX_pair_t encoder_pairs[5];
	double encoder_value = 0.0;
	double valueList[5];
	spinner.start();
	can_rx.subscribe();
	ros::Duration(1.0).sleep();
//	ros::spinOnce();

	for(int j = 0; j < 5; j++){
		encoder_pairs[j] = can_rx.getData(encoder_IDs[j]);

		for(int k =0; k < 4; k++) {
			printf("%02X", encoder_pairs[j].data[k]);
		}
		printf("\n");


		encoder_value = 0;			
		printf("%X: ", encoder_pairs[j].id);

		if(encoder_pairs[j].id == 0x181 || encoder_pairs[j].id == 0x184){
			int count = 6;
				for(int k=0;k<4;k++) {
					printf("%02X", encoder_pairs[j].data[k]);
					encoder_value += double(encoder_pairs[j].data[3-k]*pow(16,count));
					count -= 2;
				}
			
			encoder_value = (encoder_value)*0.001; //0.001 for mm to m conversion - accurate as of 31/12/18 (code or 				object dictionary options for zeroing this value at full retraction - see datasheet for relevant info
			printf(", %f.", round(encoder_value*100000.0)/100000.0);
		}
		else{
			double encoder1_value = 0.0;
			double encoder2_value = 0.0;
				int count = 2;
				for(int k=0;k<2;k++) {
					printf("%02X", encoder_pairs[j].data[k]);
					encoder1_value += double(encoder_pairs[j].data[k]*pow(16,count));
					encoder2_value += double(encoder_pairs[j].data[k+2]*pow(16,count));
					count -= 2;
				}	
				encoder_value = 	(encoder1_value+encoder2_value)/2.0;
				encoder_value = encoder_value*0.1; 
				printf(", %f.", round(encoder_value*100000.0)/100000.0); 
				if(encoder_value == 6553.5){
					std::cout<<encoder_IDs[j]<<" encoder error.";
					throw std::runtime_error("Check magnet mounting & positioning. Exiting for safety.");
				}
		}
		printf("\n");
		valueList[j] = round(encoder_value*100000.0)/100000.0;
	}


	nh.setParam("/encoders/joint_1", valueList[0]); nh.setParam("/encoders/joint_4", valueList[1]);
	nh.setParam("/encoders/joint_6", valueList[2]); nh.setParam("/encoders/joint_7", valueList[3]);
	nh.setParam("/encoders/joint_8", valueList[4]);
	system("rosparam dump /home/josh/workspace/src/hinowa_hardware_interface/config/record_home.yaml");
	ros::shutdown();
	return 0;
}

