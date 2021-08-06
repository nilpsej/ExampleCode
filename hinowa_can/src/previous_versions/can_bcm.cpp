#include <hinowa_can/can_bcm.h>
namespace hinowa_can
{
	can_bcm::can_bcm(int CANindex, bool virt, ros::NodeHandle nh){ //called once to initialise object..
		_nh = nh;
		_can_pub = nh.advertise<hinowa_can_msgs::frameArray>("/hinowa/CAN_RX", 1000);
	// default: check all CAN-IDs
		for (i=0; i < 2048 ;i++){ 
			do_set(i, ENABLE);
		}
	//define interface name...	
		std::string CANbus = "can";
		if(virt == true){	
			CANbus = "vcan";
		}
		CANbus += std::to_string(CANindex);
		CANbus_char = CANbus.c_str();

	//setup socket..
		if ((s = socket(PF_CAN, SOCK_DGRAM, CAN_BCM)) < 0) {
			perror("socket");
			//return 1;
		}
		addr.can_family = AF_CAN;

		strcpy(ifr.ifr_name, CANbus_char);
		ioctl(s, SIOCGIFINDEX, &ifr);

		addr.can_ifindex = ifr.ifr_ifindex;
		//addr.can_ifindex = 0; // any can interface

		printf("[CAN BCM INIT] %s opened at index %d\n", ifr.ifr_name, ifr.ifr_ifindex);

		if (connect(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			perror("connect");
			//return 1;
		}

		for (i=0; i < 2048 ;i++){ // initial BCM setup
			if (is_set(i, ENABLE)){
				rx_setup(s, i);
			}
		}

		gettimeofday(&start_tv, NULL);
		tv.tv_sec = tv.tv_usec = 0;
	}

	can_bcm::~can_bcm(){
	}

	void can_bcm::publish(){
		hinowa_can_msgs::frame RX_frame;
		hinowa_can_msgs::frameArray RX_array;
		RX_frame.header.stamp = ros::Time::now();

		for(int j=0; j<7;j++){
			for (i=0; i < 2048; i++) {
				if(sniftab[i].current.can_id == RX_ids[j]){
					RX_frame.id = sniftab[i].current.can_id;
					memcpy(&RX_frame.data, &sniftab[i].current.data, 8);
					RX_array.frames.push_back(RX_frame);
					//sniftab[id].current.data
				}
			}
		}
		_can_pub.publish(RX_array);
	}

	int can_bcm::run(){
		int running = 1;
		FD_ZERO(&rdfs);
		FD_SET(0, &rdfs);
		FD_SET(s, &rdfs);

		timeo.tv_sec  = 0;
		timeo.tv_usec = 100000 * loop;
	
		if ((ret = select(s+1, &rdfs, NULL, NULL, &timeo)) < 0) {
			running = 0;
		}

		gettimeofday(&tv, NULL);
		currcms = (tv.tv_sec - start_tv.tv_sec) * 10 + (tv.tv_usec / 100000);

//add this into a subscriber function for receiving updated data from main loop - right now depends on how often the value of data changes for how much time is shown between updates on cansniffer. See if a similar implementation to reading functions is needed once in subscriber..
		//data = tv.tv_usec*100000;
/*
		data = currcms; 
		for(i=0; i<15; i++) {
			tx_send(s, TX_ids[i], data);
		}
*/
//----//

		if (FD_ISSET(s, &rdfs))
			running &= handle_bcm(s, currcms);

		if (currcms - lastcms >= loop) {
			running &= handle_timeo(s, currcms);
			lastcms = currcms;
		}
		return running;
	}

	void can_bcm::rx_setup(int fd, int id){ //call once outside of loop to set up bcm RX cyclic process (monitors all addresses)..

		struct {
			struct bcm_msg_head msg_head;
			struct can_frame frame;
		} rxmsg;

		rxmsg.msg_head.opcode  = RX_SETUP;
		rxmsg.msg_head.can_id  = id;
		rxmsg.msg_head.flags   = RX_CHECK_DLC;
		rxmsg.msg_head.ival1.tv_sec  = 0;
		rxmsg.msg_head.ival1.tv_usec = 0;
		rxmsg.msg_head.ival2.tv_sec  = 0;
		rxmsg.msg_head.ival2.tv_usec = 0;
		rxmsg.msg_head.nframes = 1;
		U64_DATA(&rxmsg.frame) = (__u64) 0xFFFFFFFFFFFFFFFFULL;

		if (filter_id_only)
			rxmsg.msg_head.flags |= RX_FILTER_ID;

		if (write(fd, &rxmsg, sizeof(rxmsg)) < 0)
			perror("write");
	}

	void can_bcm::rx_delete (int fd, int id){ //Can be used to delete addresses that aren't used (not utilised at the moment)

		struct bcm_msg_head msg_head;

		msg_head.opcode  = RX_DELETE;
		msg_head.can_id  = id;
		msg_head.nframes = 0;

		if (write(fd, &msg_head, sizeof(msg_head)) < 0)
			perror("write");
	}
/*
	void can_bcm::tx_send(int fd, int id, int data){ //Called each loop to send out 1 message for each TX id - may have to setup timer l. RX
		struct {
			struct bcm_msg_head msg_head;
			struct can_frame frame;
		} msg;

		uint8_t hexData[8];
		int quotient;
		int num = abs(data);
		int nbytes;

		msg.msg_head.opcode  = TX_SEND;
		msg.msg_head.can_id  = id;
		//msg.msg_head.flags   = TX_CP_CAN_ID;
		msg.msg_head.nframes = 1;
		msg.frame.can_dlc    = 8;
		msg.frame.can_id     = id;

		if(data < 0) hexData[0] = 1;
		else hexData[0] = 0;

		for(int i=1;i<8;i++) {
			//printf("%X", send_frame.data[i]);
			hexData[i] = num % 16;
			quotient = num/16;	
			num = quotient;
		}

		for(int i=0;i<8;i++) {
			msg.frame.data[i] = hexData[i];
		}

		if (write(fd, &msg, sizeof(msg)) < 0)
			perror("write");
	}
*/

	int can_bcm::handle_bcm(int fd, long currcms){ //Assign new frames as they're read and store in sniftab struct..

		int nbytes, id;

		struct {
			struct bcm_msg_head msg_head;
			struct can_frame frame;
		} bmsg;

		if ((nbytes = read(fd, &bmsg, sizeof(bmsg))) < 0) {
			perror("bcm read");
			return 0; // quit
		}

		id = bmsg.msg_head.can_id;
		ioctl(fd, SIOCGSTAMP, &sniftab[id].currstamp);

		if (bmsg.msg_head.opcode != RX_CHANGED) {
			printf("received strange BCM opcode %d!\n", bmsg.msg_head.opcode);
			return 0; // quit
		}

		if (nbytes != sizeof(bmsg)) {
			printf("received strange BCM data length %d!\n", nbytes);
			return 0; // quit
		}

		sniftab[id].current = bmsg.frame;
		U64_DATA(&sniftab[id].marker) |= 
			U64_DATA(&sniftab[id].current) ^ U64_DATA(&sniftab[id].last);
		sniftab[id].timeout = (timeout)?(currcms + timeout):0;

		if (is_clr(id, DISPLAY))
			clearscreen = 1; // new entry -> new drawing 

		do_set(id, DISPLAY);
		do_set(id, UPDATE);
	
		return 1; // ok
	}

	int can_bcm::handle_timeo(int fd, long currcms){ //Update other portions of sniftab (last frame), print lines (remove some of this)..

		int i;
		int force_redraw = 0;

		if (clearscreen) {
			char startline[80];
//remove printing of header
			snprintf(startline, 79, "< cansniffer %s # l=%ld h=%ld t=%ld >", CANbus_char, loop, hold, timeout);
			printf("%s%*s",STARTLINESTR, 79-(int)strlen(STARTLINESTR), startline);
			force_redraw = 1;
			clearscreen = 0;
		}

		for (i=0; i < 2048; i++) {

			if is_set(i, ENABLE) {

					if is_set(i, DISPLAY) {

							if (is_set(i, UPDATE) || (force_redraw)){
								print_snifline(i); //uncomment for printing
								sniftab[i].hold = currcms + hold;
								do_clr(i, UPDATE);
							}
							else
								if ((sniftab[i].hold) && (sniftab[i].hold < currcms)) {
									U64_DATA(&sniftab[i].marker) = (__u64) 0;
									print_snifline(i); //uncomment for printing
									sniftab[i].hold = 0; // disable update by hold
								}

							if (sniftab[i].timeout && sniftab[i].timeout < currcms) {
								do_clr(i, DISPLAY);
								do_clr(i, UPDATE);
								clearscreen = 1; // removed entry -> new drawing next time
							}
						}
					sniftab[i].last      = sniftab[i].current;
					sniftab[i].laststamp = sniftab[i].currstamp;
				}
		}

		return 1; // ok

	}

	void can_bcm::print_snifline(int id){

		long diffsec  = sniftab[id].currstamp.tv_sec  - sniftab[id].laststamp.tv_sec;
		long diffusec = sniftab[id].currstamp.tv_usec - sniftab[id].laststamp.tv_usec;
		int dlc_diff  = sniftab[id].last.can_dlc - sniftab[id].current.can_dlc;
		int i,j;

		if (diffusec < 0)
			diffsec--, diffusec += 1000000;

		if (diffsec < 0)
			diffsec = diffusec = 0;

		if (diffsec > 10)
			diffsec = 9, diffusec = 999999;

		printf("%ld.%06ld  %3x  ", diffsec, diffusec, id);

	//print data...

		for (i=0; i<sniftab[id].current.can_dlc; i++)
				printf("%02X ", sniftab[id].current.data[i]);

		if (sniftab[id].current.can_dlc < 8)
			printf("%*s", (8 - sniftab[id].current.can_dlc) * 3, "");

		for (i=0; i<sniftab[id].current.can_dlc; i++)
			if ((sniftab[id].current.data[i] > 0x1F) && (sniftab[id].current.data[i] < 0x7F))
					putchar(sniftab[id].current.data[i]);
			else
				putchar('.');

		/*
		 * when the can_dlc decreased (dlc_diff > 0),
		 * we need to blank the former data printout
		 */
		for (i=0; i<dlc_diff; i++)
			putchar(' ');


		putchar('\n');

		U64_DATA(&sniftab[id].marker) = (__u64) 0;

	}
}


