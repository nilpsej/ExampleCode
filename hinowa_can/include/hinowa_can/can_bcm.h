#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <std_msgs/String.h>
#include <sstream>
#include <linux/types.h>
#include <linux/socket.h>

#include <ros/ros.h>
#include <linux/can.h>
#include <linux/can/bcm.h>
//#include <linux/can/raw.h>
#include <bits/stdc++.h> 
#include <hinowa_can_msgs/frame.h>
#include <hinowa_can_msgs/frameArray.h>
#include <hinowa_can_msgs/valve_control.h>
#include <hinowa_can/RX_struct.h>

#define U64_DATA(p) (*(unsigned long long*)(p)->data)

#define SETFNAME "sniffset."
#define ANYDEV   "any"

/* flags */

#define ENABLE  1 /* by filter or user */
#define DISPLAY 2 /* is on the screen */
#define UPDATE  4 /* needs to be printed on the screen */
#define CLRSCR  8 /* clear screen in next loop */

/* flags testing & setting */

#define is_set(id, flag) (sniftab[id].flags & flag)
#define is_clr(id, flag) (!(sniftab[id].flags & flag))

#define do_set(id, flag) (sniftab[id].flags |= flag)
#define do_clr(id, flag) (sniftab[id].flags &= ~flag)


#define MAXANI 8

#define ATTBOLD      "\33[1m"
#define ATTRESET "\33[0m"
#define FGRED     "\33[31m"
#define ATTCOLOR ATTBOLD FGRED

#define STARTLINESTR "X  time    ID  data ... "

#define TIMEOUT 0 /* in 10ms */
#define HOLD    0 /* in 10ms */
#define LOOP    0 /* in 10ms */

struct snif {
	int flags;
	long hold;
	long timeout;
	struct timeval laststamp;
	struct timeval currstamp;
	struct can_frame last;
	struct can_frame current;
	struct can_frame marker;
};

extern int optind, opterr, optopt;


namespace hinowa_can
{
	class can_bcm
	{
		public:
			can_bcm(int CANindex, bool virt, ros::NodeHandle nh);
			virtual ~can_bcm();
			void publish();
			int run();

			void rx_setup(int fd, int id);
			void rx_delete(int fd, int id);

			//void tx_send(int fd, int id, int data);

			int handle_bcm(int fd, long currcms);
			int handle_timeo(int fd, long currcms);
			void print_snifline(int id);

			snif sniftab[2048];
			//void rx_setup(int fd, int id);
/*
			void valveControlCallback(const hinowa_can_msgs::valve_control::ConstPtr& instruction);
			void assignData(struct can_frame frame);
			int readData();
			void publish();
*/

		private:
			ros::NodeHandle _nh;
			ros::Publisher _can_pub;
			int i;
			int running = 1;
			int clearscreen = 1;
			int filter_id_only;
			long timeout = TIMEOUT; //50ms timeout, in 100ms scaling
			long hold = HOLD; //10ms hold, in 100ms scaling
			long loop = LOOP; //4ms loop rate, in 100ms scaling
			unsigned char color;
			char const *CANbus_char;
			
//variables from main function in can_sniffer
			fd_set rdfs;
			int s;
			canid_t mask = 0;
			canid_t value = 0;
			long currcms = 0;
			long lastcms = 0;
			unsigned char quiet = 0;
			int opt, ret;
			struct timeval timeo, start_tv, tv;
			struct sockaddr_can addr;
			struct ifreq ifr;

			uint16_t RX_ids[7] = {0x89, 0x090, 0x181, 0x182, 0x183, 0x184, 0x185};

			int data; //this will be received from subscriber
			//uint16_t TX_ids[15] = {0x060, 0x091, 0x092, 0x093, 0x094, 0x095, 0x096, 0x102, 0x103, 0x104, 0x105, 0x106, 0x198, 0x199, 0x200}; //CHANGE THIS TO BE AN INPUT VARIABLE EITHER IN THE FORM OF A SINGLE ID OR AS AN ARRAY FROM THE FUNCTION THATS INSTRUCTING THE CALL COMMAND (IDEALLY EACH ID WILL BE CALLED TO SEND INDIVIDUALLY - WILL HAVE  TO CHANGE MAIN LOOP TO DO SO)
	};
}

#endif
