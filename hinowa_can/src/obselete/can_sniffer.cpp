#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>
#include <bits/stdc++.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/bcm.h>

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

struct snif {
	int flags;
	long hold;
	long timeout;
	struct timeval laststamp;
	struct timeval currstamp;
	struct can_frame last;
	struct can_frame current;
	struct can_frame marker;
} sniftab[2048];


extern int optind, opterr, optopt;

static int running = 1;
static int clearscreen = 1;
static int filter_id_only;
static long timeout = 0.0; //50ms timeout, in 100ms scaling
static long hold = 0.0; //10ms hold, in 100ms scaling
static long loop = 0.0; //4ms loop rate, in 100ms scaling

static unsigned char color;
static char *interface;

void rx_setup (int fd, int id);
void rx_delete (int fd, int id);

void tx_send(int fd, int id, int data);

int handle_bcm(int fd, long currcms);
int handle_timeo(int fd, long currcms);
void print_snifline(int id);

	
int main(int argc, char **argv)
{
	system("sudo ip link add dev vcan0 type vcan");
	system("sudo ip link set up vcan0");

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
	int i;

	for (i=0; i < 2048 ;i++) // default: check all CAN-IDs
		do_set(i, ENABLE);

//define interface name...
	char interface_char[] = "vcan0";
	interface = interface_char;

//setup socket..
	if ((s = socket(PF_CAN, SOCK_DGRAM, CAN_BCM)) < 0) {
		perror("socket");
		return 1;
	}
	addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, "vcan0");
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_ifindex = ifr.ifr_ifindex;
	//addr.can_ifindex = 0; // any can interface

	if (connect(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("connect");
		return 1;
	}

	for (i=0; i < 2048 ;i++) // initial BCM setup
		if (is_set(i, ENABLE))
			rx_setup(s, i);
	
	uint16_t TX_ids[15] = {0x060, 0x091, 0x092, 0x093, 0x094, 0x095, 0x096, 0x102, 0x103, 0x104, 0x105, 0x106, 0x198, 0x199, 0x200};
	int data;
/*	int data = 10000;
	for(i=0; i<15; i++) {
		tx_send(s, TX_ids[i], data);
	}
*/

	gettimeofday(&start_tv, NULL);
	tv.tv_sec = tv.tv_usec = 0;

	while (running) {

		FD_ZERO(&rdfs);
		FD_SET(0, &rdfs);
		FD_SET(s, &rdfs);

		timeo.tv_sec  = 0;
		timeo.tv_usec = 100000 * loop;
	
		if ((ret = select(s+1, &rdfs, NULL, NULL, &timeo)) < 0) {
			running = 0;
			continue;
		}


		gettimeofday(&tv, NULL);
		currcms = (tv.tv_sec - start_tv.tv_sec) * 10 + (tv.tv_usec / 100000);

//add this into a subscriber function for receiving updated data from main loop - right now depends on how often the value of data changes for how much time is shown between updates on cansniffer. See if a similar implementation to reading functions is needed once in subscriber..
		//data = tv.tv_usec*100000;
		data = currcms; 
		for(i=0; i<15; i++) {
			tx_send(s, TX_ids[i], data);
		}
//----//

		if (FD_ISSET(s, &rdfs))
			running &= handle_bcm(s, currcms);

		if (currcms - lastcms >= loop) {
			running &= handle_timeo(s, currcms);
			lastcms = currcms;
		}
	}

	close(s);
	return 0;
}

void rx_setup (int fd, int id){

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
};

void rx_delete (int fd, int id){

	struct bcm_msg_head msg_head;

	msg_head.opcode  = RX_DELETE;
	msg_head.can_id  = id;
	msg_head.nframes = 0;

	if (write(fd, &msg_head, sizeof(msg_head)) < 0)
		perror("write");
}

void tx_send(int fd, int id, int data){
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

/*Cyclic sending..cannot adjust data
void tx_setup (int fd, int id, int data){

	struct {
		struct bcm_msg_head msg_head;
		struct can_frame frame;
	} msg;

	uint8_t hexData[8];
	int quotient;
	int num = abs(data);
	int nbytes;

	//send_frame.can_id = instruction->ids[i];
	///send_frame.can_dlc = 8;

	msg.msg_head.opcode  = TX_SETUP;
	msg.msg_head.can_id  = id;
	msg.msg_head.flags   = SETTIMER|STARTTIMER|TX_CP_CAN_ID;
	msg.msg_head.nframes = 1;
	msg.msg_head.count = 0;
	msg.msg_head.ival1.tv_sec = 0;
	msg.msg_head.ival1.tv_usec = 0;
	msg.msg_head.ival2.tv_sec = 0;
	msg.msg_head.ival2.tv_usec = 100000;
	msg.frame.can_dlc   = 8;

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


int handle_bcm(int fd, long currcms){

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
};

int handle_timeo(int fd, long currcms){

	int i;
	int force_redraw = 0;

	if (clearscreen) {
		char startline[80];
		snprintf(startline, 79, "< cansniffer %s # l=%ld h=%ld t=%ld >", interface, loop, hold, timeout);
		printf("%s%*s",STARTLINESTR, 79-(int)strlen(STARTLINESTR), startline);
		force_redraw = 1;
		clearscreen = 0;
	}

	for (i=0; i < 2048; i++) {

		if is_set(i, ENABLE) {

				if is_set(i, DISPLAY) {

						if (is_set(i, UPDATE) || (force_redraw)){
							print_snifline(i);
							sniftab[i].hold = currcms + hold;
							do_clr(i, UPDATE);
						}
						else
							if ((sniftab[i].hold) && (sniftab[i].hold < currcms)) {
								U64_DATA(&sniftab[i].marker) = (__u64) 0;
								print_snifline(i);
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

};

void print_snifline(int id){

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

};
