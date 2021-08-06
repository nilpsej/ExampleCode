#include <hinowa_can/can_raw.h>
#include <std_msgs/String.h>

#include <sstream>
#include <linux/types.h>
#include <linux/socket.h>


namespace hinowa_can
{
	can_raw::can_raw(int bus, bool virt, ros::NodeHandle nh){
		_CANbus = bus;
		_virt = virt;
		_nh = nh;
		openCAN();
		
		for(int i=0; i<15; i++){
			prev_TX_pairs[i].id = TX_ids[i];
			prev_TX_pairs[i].data = 1; //initialise as non zero so that when compared on startup, 0s still get sent once. Replaced from there after comparison.
		}
		_can_sub = _nh.subscribe<hinowa_can_msgs::valve_control>("/hinowa/CAN_TX", 1000, &can_raw::valveControlCallback, this);
	}

	can_raw::~can_raw(){
		//close(CANsocket);
		//printf("~reached.\n");
	}


	void can_raw::valveControlCallback(const hinowa_can_msgs::valve_control::ConstPtr& instruction){//CAN WRITE, automatically called. 
		for(int i=0; i<15; i++){
//WRITE TO CAN CODE
			uint8_t hexData[8];
			int quotient;
			int num = abs(instruction->data[i]);
			int nbytes;	
			//socklen_t len = sizeof(addr);	
			struct can_frame send_frame;
			send_frame.can_id = instruction->ids[i];
			send_frame.can_dlc = 8;
		
			if(instruction->data[i] < 0) hexData[0] = 1;
			else hexData[0] = 0;

			for(int i=1;i<8;i++) {
				//printf("%X", send_frame.data[i]);
				hexData[i] = num % 16;
				quotient = num/16;	
				num = quotient;
			}

			for(int i=0;i<8;i++) {
				send_frame.data[i] = hexData[i];
			}

			//printf("ID: %X.\n", instruction->ids[i]);
			if((send_frame.can_id == 0x198) || (send_frame.can_id == 0x199) || (send_frame.can_id == 0x200)){
				//printf("ID: %X.\n", send_frame.can_id);
				nbytes = write(CANsocket, &send_frame, sizeof(struct can_frame)); 
			}
			//for valve heartbeats & block selection IDs, always send. 
			else{
				if(prev_TX_pairs[i].data != instruction->data[i]) nbytes = write(CANsocket, &send_frame, sizeof(struct can_frame));
				else nbytes = 1;

			}
			//nbytes = write(CANsocket, &send_frame, sizeof(struct can_frame)); //swap for above when onbaord pcb can reliably receive one message sends

//Data write check
			if(nbytes < 0) {
			// Error: no data written, exit for safety.
				printf("Nbytes is less than 0: %i, failed to send CAN data.\n",nbytes);
				throw std::runtime_error("hinowa_can/can_interface.cpp: (Socketcan) Could not write data to CAN-bus.\n");
			}
			//else printf("Sent %X.\n", send_frame.can_id);

			prev_TX_pairs[i].data = instruction->data[i];
		//printf("|CAN frame written| ID: %X, Data: %d.\n", address, data);
		}

		
	}

	int can_raw::openCAN(){
		std::string CANbus = "can";
		if(_virt == true){	
			CANbus = "vcan";
		}
		CANbus += std::to_string(_CANbus);
		char const* CANbus_char = CANbus.c_str();

		if((CANsocket = socket(PF_CAN, SOCK_RAW, CAN_RAW))<0){
		ROS_ERROR("Error while opening socket");
		throw std::runtime_error("CAN socket error while opening. Exiting for safety.");
		return -1;
		}
		//setup code (dont modify)
		strcpy(ifr.ifr_name, CANbus_char);
		ioctl(CANsocket, SIOCGIFINDEX, &ifr);
	
		addr.can_family  = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;

		if(bind(CANsocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			ROS_ERROR("Error in socket bind");
			throw std::runtime_error("CAN error while binding. Exiting for safety.");
			return -1;	
		}
		else printf("[CAN TX INIT] %s opened at index %d\n", ifr.ifr_name, ifr.ifr_ifindex);

	}


}


