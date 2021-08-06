#include <hinowacpp/can_rx_subscriber.h>

namespace hinowacpp
{

	CAN_RX_SUBSCRIBER::CAN_RX_SUBSCRIBER(){
	}

	CAN_RX_SUBSCRIBER::CAN_RX_SUBSCRIBER(ros::NodeHandle nh){
		this->_nh = nh;

		for(int i=0; i<n; i++){
			RX_pairs[i].id = RX_ids[i];
		}
	}

	CAN_RX_SUBSCRIBER::~CAN_RX_SUBSCRIBER(){
	}

	void CAN_RX_SUBSCRIBER::storeData(const hinowa_can_msgs::frameArray::ConstPtr& rx_frames){
		hinowa_can_msgs::frame rx_frame;
		for(int i=0; i<n; i++){
			rx_frame = rx_frames->frames[i];
			if(RX_pairs[i].id == rx_frame.id) memcpy(&RX_pairs[i].data, &rx_frame.data, 8);
		}
/*
		for(int i=0;i<4;i++) {
			printf("%02X", RX_pairs[4].data[i]);
		}
		printf("\n");
*/
	}


	//RX_pairs_t* CAN_RX_SUBSCRIBER::subscribe(){
	void CAN_RX_SUBSCRIBER::subscribe(){
		this->_sub = _nh.subscribe<hinowa_can_msgs::frameArray>("/hinowa/CAN_RX", 1000, &CAN_RX_SUBSCRIBER::storeData, this);
	}

	RX_pair_t CAN_RX_SUBSCRIBER::getData(uint16_t ID){
		RX_pair_t requested_pair;
		for(int i=0; i<n; i++){
			if(RX_pairs[i].id == ID) requested_pair = RX_pairs[i];
		}
/*
		printf("Address: %X, data: ", requested_pair.id);
		for(int i=0;i<4;i++) {
			printf("%02X", requested_pair.data[i]);
		}
		printf("\n");
*/
		return requested_pair;
	}

}
