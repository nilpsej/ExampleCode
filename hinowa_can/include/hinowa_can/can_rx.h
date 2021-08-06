#ifndef CAN_RX_H
#define CAN_RX_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>

#include "hinowa_can/terminal.h"
#include <ros/ros.h>
/*
#include <hinowa_can_msgs/frame.h>
#include <hinowa_can_msgs/frameArray.h>
#include <hinowa_can_msgs/valve_control.h>
#include <hinowa_can/RX_struct.h>
*/
#define SETFNAME "sniffset."
#define ANYDEV   "any"
#define MAX_SLOTS 2048

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

#define ATTCOLOR ATTBOLD FGRED

#define LDL " | "	/* long delimiter */
#define SDL "|"		/* short delimiter for binary on 80 chars terminal */

struct snif {
	int flags;
	long hold;
	long timeout;
	struct timeval laststamp;
	struct timeval currstamp;
	struct can_frame last;
	struct can_frame current;
	struct can_frame marker;
	struct can_frame notch;
};

extern int optind, opterr, optopt;
static int running = 1;

namespace hinowa_can
{
	class can_rx
	{
		public:
			can_rx(int CANindex, bool virt, ros::NodeHandle nh);
			virtual ~can_rx();
			//void publish();
			int run();
			//void switchvdl(char *delim);
			static int comp(const void *elem1, const void *elem2);
			void do_modify_sniftab(unsigned int value, unsigned int mask, char cmd);
			int handle_frame(int fd, long currcms);
			int handle_timeo(long currcms);
			void print_snifline(int slot);
			void writesettings(char* name);
			int readsettings(char* name);
			int sniftab_index(canid_t id);

			//void rx_setup(int fd, int id);
			//void rx_delete(int fd, int id);


			//int handle_bcm(int fd, long currcms);
			//int handle_timeo(int fd, long currcms);
			//void print_snifline(int id);

			//snif sniftab[2048];
			snif sniftab[MAX_SLOTS];


		private:
			ros::NodeHandle _nh;
			ros::Publisher _can_pub;

			int i,j;
			int clearscreen = 1;
			int idx;
			int print_eff = 0;
			int notch;
			long timeout = 0; //50ms timeout, in 100ms scaling
			long hold = 0; //10ms hold, in 100ms scaling
			long loop = 0; //4ms loop rate, in 100ms scaling
			unsigned char binary;
			unsigned char binary8;
			unsigned char binary_gap;
			unsigned char color;
			char const *interface;
			char const *vdl; /* variable delimiter */
			char const *ldl; /* long delimiter */
	
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

			//uint16_t RX_ids[7] = {0x89, 0x090, 0x181, 0x182, 0x183, 0x184, 0x185};
	};
}

#endif
