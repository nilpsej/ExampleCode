#include <hinowa_can/can_rx.h>

void sigterm(int signo)
{
	running = 0;
}


namespace hinowa_can
{
	can_rx::can_rx(int CANindex, bool virt, ros::NodeHandle nh){ //called once to initialise object..

		signal(SIGTERM, sigterm);
		signal(SIGHUP, sigterm);
		signal(SIGINT, sigterm);


		for (i = 0; i < MAX_SLOTS ;i++) // default: enable all slots
		do_set(i, ENABLE);

		//define interface name...	
		std::string CANbus = "can";
		if(virt == true){	
			CANbus = "vcan";
		}
		CANbus += std::to_string(CANindex);
		CANbus_char = CANbus.c_str();
		interface = CANbus_char;

		std::string ldl_str = "LDL";
		ldl = ldl_str.c_str();
		vdl = ldl;

		//setup socket

		s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (s < 0) {
			//perror("socket");
			throw std::runtime_error("hinowa_can/can_rx - Socket setup error.\n");
		}

		addr.can_family = AF_CAN;

		strcpy(ifr.ifr_name, CANbus_char);
		ioctl(s, SIOCGIFINDEX, &ifr);

		addr.can_ifindex = ifr.ifr_ifindex;

		if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			//perror("connect");
			throw std::runtime_error("hinowa_can/can_rx - Socket binding error.\n");
		}

		gettimeofday(&start_tv, NULL);
		tv.tv_sec = tv.tv_usec = 0;

		if(verbose) printf("%s", CSR_HIDE); // hide cursor 

	}

	can_rx::~can_rx(){
	}

/*
	void can_rx::publish(){
		//ros publish to topic CAN_RX
	}
*/
/*
	void sigterm(int signo)
	{
		//running = 0;
	}
*/
	int can_rx::run()
	{
		//int running = 1;

		FD_ZERO(&rdfs);
		FD_SET(0, &rdfs);
		FD_SET(s, &rdfs);

		timeo.tv_sec  = 0;
		timeo.tv_usec = 10000 * loop;

		if ((ret = select(s+1, &rdfs, NULL, NULL, &timeo)) < 0) {
			//perror("select");
			running = 0;
		}

		gettimeofday(&tv, NULL);
		currcms = (tv.tv_sec - start_tv.tv_sec) * 100 + (tv.tv_usec / 10000);
/*
		if (FD_ISSET(0, &rdfs))
			running &= handle_keyb();
*/
		if (FD_ISSET(s, &rdfs))
			running &= handle_frame(s, currcms);

		if (currcms - lastcms >= loop) {
			running &= handle_timeo(currcms);
			lastcms = currcms;
		}

		if(!running){
			printf("%s", CSR_SHOW); // show cursor
			close(s);
			return 0;
		}
		//printf("%s", CSR_SHOW); // show cursor
		//close(s);
		return running;
	}
/*
	void can_rx::switchvdl(char *delim)
	{
		// reduce delimiter size for EFF IDs in binary display of up to 8 data bytes payload to fit into 80 chars per line 
		if (binary8)
			vdl = delim;
	}
*/

	int can_rx::comp(const void *elem1, const void *elem2)
	{
	    unsigned long f = ((struct snif*)elem1)->current.can_id;
	    unsigned long s = ((struct snif*)elem2)->current.can_id;

	    if (f > s)
		    return  1;
	    if (f < s)
		    return -1;

	    return 0;
	}

	void can_rx::do_modify_sniftab(unsigned int value, unsigned int mask, char cmd)
	{
		for (i = 0; i < idx ;i++) {
			if ((sniftab[i].current.can_id & mask) == (value & mask)) {
				if (cmd == '+')
					do_set(i, ENABLE);
				else
					do_clr(i, ENABLE);
			}
		}
	}

	int can_rx::handle_frame(int fd, long currcms)
	{
		bool rx_changed = false;
		bool run_qsort = false;
		int nbytes, i, pos;
		struct can_frame cf;

		if ((nbytes = read(fd, &cf, sizeof(cf))) < 0) {
			perror("raw read");
			return 0; /* quit */
		}

		if (nbytes != CAN_MTU) {
			printf("received strange frame data length %d!\n", nbytes);
			return 0; /* quit */
		}

		if (!print_eff && (cf.can_id & CAN_EFF_FLAG)) {
			print_eff = 1;
			clearscreen = 1;
		}

		pos = sniftab_index(cf.can_id);
		if (pos < 0) {
			/* CAN ID not existing */
			if (idx < MAX_SLOTS) {
				/* assign new slot */
				pos = idx++;
				rx_changed = true;
				run_qsort = true;
			} else {
				/* informative exit */
				perror("number of different CAN IDs exceeded MAX_SLOTS");
				return 0; /* quit */
			}
		}
		else {
			if (cf.can_dlc == sniftab[pos].current.can_dlc)
				for (i = 0; i < cf.can_dlc; i++) {
					if (cf.data[i] != sniftab[pos].current.data[i] ) {
						rx_changed = true;
						break;
					}
				}
			else
				rx_changed = true;
		}

		/* print received frame even if the data didn't change to get a gap time */
		if ((sniftab[pos].laststamp.tv_sec == 0) && (sniftab[pos].laststamp.tv_usec == 0))
			rx_changed = true;

		if (rx_changed == true) {
			sniftab[pos].laststamp = sniftab[pos].currstamp;
			ioctl(fd, SIOCGSTAMP, &sniftab[pos].currstamp);

			sniftab[pos].current = cf;
			for (i = 0; i < 8; i++)
				sniftab[pos].marker.data[i] |= sniftab[pos].current.data[i] ^ sniftab[pos].last.data[i];

			sniftab[pos].timeout = (timeout)?(currcms + timeout):0;

			if (is_clr(pos, DISPLAY))
				clearscreen = 1; /* new entry -> new drawing */

			do_set(pos, DISPLAY);
			do_set(pos, UPDATE);
		}

		if (run_qsort == true)
			qsort(sniftab, idx, sizeof(sniftab[0]), comp);

		return 1; /* ok */
	};

	int can_rx::handle_timeo(long currcms)
	{
		int force_redraw = 0;
		static unsigned int frame_count;

		if (clearscreen) {

			if (print_eff)
				printf("%s%sXX|ms%s-- ID --%sdata ...     < %s # l=%ld h=%ld t=%ld slots=%d >",
				       CLR_SCREEN, CSR_HOME, vdl, vdl, interface, loop, hold, timeout, idx);

			else

				printf("%s%sXX|ms%sID %sdata ...     < %s # l=%ld h=%ld t=%ld slots=%d >",
				       CLR_SCREEN, CSR_HOME, ldl, ldl, interface, loop, hold, timeout, idx);

			force_redraw = 1;
			clearscreen = 0;
		}

		if (notch) {
			for (i = 0; i < idx; i++) {
				for (j = 0; j < 8; j++)
					sniftab[i].notch.data[j] |= sniftab[i].marker.data[j];
			}
			notch = 0;
		}

		printf("%s", CSR_HOME);
		printf("%02d\n", frame_count++); /* rolling display update counter */
		frame_count %= 100;

		for (i = 0; i < idx; i++) {
			if is_set(i, ENABLE) {
					if is_set(i, DISPLAY) {
							if (is_set(i, UPDATE) || (force_redraw)) {
								print_snifline(i);
								sniftab[i].hold = currcms + hold;
								do_clr(i, UPDATE);
							}
							else  if ((sniftab[i].hold) && (sniftab[i].hold < currcms)) {
									memset(&sniftab[i].marker.data, 0, 8);
									print_snifline(i);
									sniftab[i].hold = 0; /* disable update by hold */
								}
							else
								printf("%s", CSR_DOWN); /* skip my line */

							if (sniftab[i].timeout && sniftab[i].timeout < currcms) {
								do_clr(i, DISPLAY);
								do_clr(i, UPDATE);
								clearscreen = 1; /* removed entry -> new drawing next time */
							}
						}
					sniftab[i].last      = sniftab[i].current;
				}
		}

		return 1; /* ok */
	};

	void can_rx::print_snifline(int slot)
	{
		long diffsec  = sniftab[slot].currstamp.tv_sec  - sniftab[slot].laststamp.tv_sec;
		long diffusec = sniftab[slot].currstamp.tv_usec - sniftab[slot].laststamp.tv_usec;
		int dlc_diff  = sniftab[slot].last.can_dlc - sniftab[slot].current.can_dlc;
		canid_t cid = sniftab[slot].current.can_id;

		if (diffusec < 0)
			diffsec--, diffusec += 1000000;

		if (diffsec < 0)
			diffsec = diffusec = 0;

		if (diffsec >= 100)
			diffsec = 99, diffusec = 999999;

		if (cid & CAN_EFF_FLAG)
			printf("%02ld%03ld%s%08X%s", diffsec, diffusec/1000, vdl, cid & CAN_EFF_MASK, vdl);
		else if (print_eff)
			printf("%02ld%03ld%s---- %03X%s", diffsec, diffusec/1000, vdl, cid & CAN_SFF_MASK, vdl);
		else
			printf("%02ld%03ld%s%03X%s", diffsec, diffusec/1000, ldl, cid & CAN_SFF_MASK, ldl);

		if (binary) {
			for (i = 0; i < sniftab[slot].current.can_dlc; i++) {
				for (j=7; j >= 0; j--) {
					if ((color) && (sniftab[slot].marker.data[i] & 1<<j) &&
					    (!(sniftab[slot].notch.data[i] & 1<<j)))
						if (sniftab[slot].current.data[i] & 1<<j)
							printf("%s1%s", ATTCOLOR, ATTRESET);
						else
							printf("%s0%s", ATTCOLOR, ATTRESET);
					else
						if (sniftab[slot].current.data[i] & 1<<j)
							putchar('1');
						else
							putchar('0');
				}
				if (binary_gap)
					putchar(' ');
			}

			/*
			 * when the can_dlc decreased (dlc_diff > 0),
			 * we need to blank the former data printout
			 */
			for (i = 0; i < dlc_diff; i++) {
				printf("        ");
				if (binary_gap)
					putchar(' ');
			}
		}
		else {
			for (i = 0; i < sniftab[slot].current.can_dlc; i++)
				if ((color) && (sniftab[slot].marker.data[i] & ~sniftab[slot].notch.data[i]))
					printf("%s%02X%s ", ATTCOLOR, sniftab[slot].current.data[i], ATTRESET);
				else
					printf("%02X ", sniftab[slot].current.data[i]);

			if (sniftab[slot].current.can_dlc < 8)
				printf("%*s", (8 - sniftab[slot].current.can_dlc) * 3, "");

			for (i = 0; i<sniftab[slot].current.can_dlc; i++)
				if ((sniftab[slot].current.data[i] > 0x1F) &&
				    (sniftab[slot].current.data[i] < 0x7F))
					if ((color) && (sniftab[slot].marker.data[i] & ~sniftab[slot].notch.data[i]))
						printf("%s%c%s", ATTCOLOR, sniftab[slot].current.data[i], ATTRESET);
					else
						putchar(sniftab[slot].current.data[i]);
				else
					putchar('.');

			/*
			 * when the can_dlc decreased (dlc_diff > 0),
			 * we need to blank the former data printout
			 */
			for (i = 0; i < dlc_diff; i++)
				putchar(' ');
		}

		putchar('\n');

		memset(&sniftab[slot].marker.data, 0, 8);
	};

	void can_rx::writesettings(char* name)
	{
		int fd;
		char fname[30] = SETFNAME;
		char buf[13]= {0};

		strncat(fname, name, 29 - strlen(fname)); 
		fd = open(fname, O_WRONLY|O_CREAT, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
	    
		if (fd > 0) {
			for (i = 0; i < idx ;i++) {
				sprintf(buf, "<%08X>%c.", sniftab[i].current.can_id, (is_set(i, ENABLE))?'1':'0');
				if (write(fd, buf, 12) < 0)
					perror("write");
				for (j = 0; j < 8 ; j++) {
					sprintf(buf, "%02X", sniftab[i].notch.data[j]);
					if (write(fd, buf, 2) < 0)
						perror("write");
				}
				if (write(fd, "\n", 1) < 0)
					perror("write");
				/* 12 + 16 + 1 = 29 bytes per entry */
			}
			close(fd);
		}
		else
			printf("unable to write setting file '%s'!\n", fname);
	};

	int can_rx::readsettings(char* name)
	{
		int fd;
		char fname[30] = SETFNAME;
		char buf[30] = {0};
		bool done = false;

		strncat(fname, name, 29 - strlen(fname)); 
		fd = open(fname, O_RDONLY);
	    
		if (fd > 0) {
			idx = 0;
			while (!done) {
				if (read(fd, &buf, 29) == 29) {
					unsigned long id = strtoul(&buf[1], (char **)NULL, 16);

					sniftab[idx].current.can_id = id;

					if (buf[10] & 1)
						do_set(idx, ENABLE);
					else
						do_clr(idx, ENABLE);

					for (j = 7; j >= 0 ; j--) {
						sniftab[idx].notch.data[j] =
							(__u8) strtoul(&buf[2*j+12], (char **)NULL, 16) & 0xFF;
						buf[2*j+12] = 0; /* cut off each time */
					}

					if (++idx >= MAX_SLOTS)
						break;
				}
				else
					done = true;
			}
			close(fd);
		}
		else
			return -1;

		return idx;
	};

	int can_rx::sniftab_index(canid_t id)
	{

		for (i = 0; i <= idx; i++)
			if (id == sniftab[i].current.can_id)
				return i;

		return -1; /* No match */
	}
}
