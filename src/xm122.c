#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <byteswap.h>

#define MODE_ENVELOPE  2
#define MODE_IQ 3



// Signal handler sets this to 1 when signal received.
// Monitored by data dump loop to exit.
int stop_signal = 0;

int uart_set_interface_attribs (int fd, int speed, int parity) {
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf (stderr, "error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                fprintf (stderr, "error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void uart_set_blocking (int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf (stderr, "error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                fprintf (stderr, "error %d setting term attributes", errno);
}

void register_write (int device, int address, uint32_t value) {

	struct __attribute__((__packed__))  {
		uint8_t start_marker;
		uint16_t payload_len;
		uint8_t packet_type;
		uint8_t address;
		uint32_t value;
		uint8_t end_marker;
	} register_write_frame;

	register_write_frame.start_marker = 0xCC;
	register_write_frame.payload_len = 5;
	register_write_frame.packet_type = 0xF9;
	register_write_frame.address = address;
	register_write_frame.value = value;
	register_write_frame.end_marker = 0xCD;

	write (device,&register_write_frame,sizeof(register_write_frame));

	// Read the write register response back (10 bytes) and ignore
	uint8_t buf[10];
	read (device,buf,10);
}

void register_read (int device, int address, uint32_t *value) {

	uint8_t buf[] = {0xCC, 0x01, 0x00, 0xF8, address, 0xCD};

	write (device,buf, 6);

	struct __attribute__((__packed__))  {
		uint8_t start_marker;
		uint16_t payload_len;
		uint8_t packet_type;
		uint8_t address;
		uint32_t value;
		uint8_t end_marker;
	} register_read_response;

	read (device, &register_read_response, sizeof(register_read_response));
	
	/*
	uint8_t *rbuf = (uint8_t *) &register_read_response;
	for (int i = 0; i < 10; i++) {
		fprintf (stdout, " %02x ", rbuf[i]);
	}
	fprintf (stdout, "\n");
	fprintf (stdout,"v=%x\n",register_read_response.value);
	*/
	*value = register_read_response.value;
}

/**
 * Download values of registers and print to stdout.
 */
void dump_registers (int device) {
	uint32_t value;
	for (int i = 0x2; i <= 0x29; i++) {
		register_read (device, i, &value);
		fprintf(stdout,"REG[%02x]=%08x\n", i, value);
	}
}

void peak_service (int device) {
	register_write (device, 0x02, 0x100);
	register_write (device, 0x20, 0x41);
	register_write (device, 0x21, 0x96);
	register_write (device, 0x03, 0x03);

	uint32_t value;

	for (int i = 0; i < 100; i++) {

		register_write (device, 0x03, 0x04);

		do {
			fprintf (stdout,"*");
			register_read(device, 0x06, &value);
		} while (value&0x100 == 0);

		int npeak;
		register_read(device, 0xB0, &npeak);
		fprintf (stdout,"npeak=%d\n", npeak);
	}

	register_write (device, 0x03, 0x00);
}

void dump_envelope_data (int device) {

	struct __attribute__((__packed__))  {
		uint16_t payload_len;
		uint8_t packet_type;
	} stream_frame_header;

	uint8_t buf[4096];

 	uint8_t b;
	struct timespec timestamp; 

	while ( ! stop_signal ) {

		// seek start of frame
		do {
			read(device, &b, 1);
		} while (b != 0xCC);

		// timestamp of reception of start of frame 
		clock_gettime(CLOCK_REALTIME, &timestamp);

		read(device, &stream_frame_header, sizeof(stream_frame_header));
		if (stream_frame_header.packet_type == 0xFE) {
			int n = 0;

			while (n <  stream_frame_header.payload_len) {
				n += read(device, buf+n, stream_frame_header.payload_len - n);
			}

			uint16_t *v;
			// timestamp to ms resolution
			fprintf (stdout, "%ld.%03ld", timestamp.tv_sec, timestamp.tv_nsec/1000000);
			for (int i = 12; i < stream_frame_header.payload_len; i+=2) {
				v = (uint16_t *) &buf[i];
				fprintf (stdout, " %4d", *v);
			}
			fprintf(stdout,"\n");

		} 
	}
	fprintf (stderr,"stop signal received\n");
}

/**
 * Data sheet seems to be wrong: complex numbers respresented by pair of int16_t.
 */
void dump_iq_data (int device) {

	struct __attribute__((__packed__))  {
		uint16_t payload_len;
		uint8_t packet_type;
	} stream_frame_header;

	uint8_t buf[4096];

 	uint8_t b;
	struct timespec timestamp; 

	while ( ! stop_signal ) {

		// seek start of frame
		do {
			read(device, &b, 1);
		} while (b != 0xCC);

		// timestamp of reception of start of frame 
		clock_gettime(CLOCK_REALTIME, &timestamp);

		read(device, &stream_frame_header, sizeof(stream_frame_header));
		if (stream_frame_header.packet_type == 0xFE) {
			int n = 0;

			while (n <  stream_frame_header.payload_len) {
				n += read(device, buf+n, stream_frame_header.payload_len - n);
			}

			int16_t *v;
			// timestamp to ms resolution
			fprintf (stdout, "%ld.%03ld", timestamp.tv_sec, timestamp.tv_nsec/1000000);
			for (int i = 12; i < stream_frame_header.payload_len; i+=2) {
				v = (int16_t *) &buf[i];
				fprintf (stdout, " %4d", *v);
			}
			fprintf(stdout,"\n");

		} 
	}
	fprintf (stderr,"stop signal received\n");
}


float ieee_float(uint32_t f)
{
    union int_float{
        uint32_t i;
        float f;
    } tofloat;

    tofloat.i = f;
    return tofloat.f;
}

void dump_float_iq_data (int device) {

	struct __attribute__((__packed__))  {
		uint16_t payload_len;
		uint8_t packet_type;
	} stream_frame_header;

	uint8_t buf[4096];

 	uint8_t b;
	struct timespec timestamp; 

	while ( ! stop_signal ) {

		// seek start of frame
		do {
			read(device, &b, 1);
		} while (b != 0xCC);

		// timestamp of reception of start of frame 
		clock_gettime(CLOCK_REALTIME, &timestamp);

		read(device, &stream_frame_header, sizeof(stream_frame_header));
		if (stream_frame_header.packet_type == 0xFE) {
			int n = 0;

			while (n <  stream_frame_header.payload_len) {
				n += read(device, buf+n, stream_frame_header.payload_len - n);
			}

			for (int i = 0; i <  stream_frame_header.payload_len; i++) {
				fprintf (stderr," %02x", buf[i]);
			}
			fprintf (stderr,"\n");

			uint32_t *v = (uint32_t *)&buf[16];
			uint32_t vs;
			float *f;
			float fs,ff;
			// timestamp to ms resolution
			fprintf (stdout, "%ld.%03ld", timestamp.tv_sec, timestamp.tv_nsec/1000000);
			for (int i = 12; i < stream_frame_header.payload_len; i+=4) {
				vs = __bswap_32(*v);
				f = (float *)v;
				ff = ieee_float(*v);
				fs = ieee_float(vs);
		//		fprintf (stdout, " %08x %08x %e %e | ", *v, vs, ff, fs );
				fprintf (stdout, " %e ", fs );
				v++;
			}
			fprintf(stdout,"\n");

		}
	}
	fprintf (stderr,"stop signal received\n");
}

void envelope_service (int device) {
	// envelope service mode
	register_write (device, 0x02, 0x02);

	// range from 65mm 
	register_write (device, 0x20, 0x41);
	// range length 150mm (from 65mm to 215mm)
	register_write (device, 0x21, 0x96);

	// start streaming
	register_write (device, 0x05, 0x01);

	register_write (device, 0x03, 0x03);

	dump_envelope_data (device);

	// stop
	register_write (device, 0x03, 0x00);
}

void iq_service (int device) {
	// IQ service mode
	register_write (device, 0x02, 0x03);

	// set range
	register_write (device, 0x20, 0x41);
	register_write (device, 0x21, 0x96);

	// start streaming
	register_write (device, 0x05, 0x01);

	register_write (device, 0x03, 0x03);

	//dump_iq_data (device);
	dump_iq_data (device);

	// stop
	register_write (device, 0x03, 0x00);
}



/**
 * Signal handler for handling SIGPIPE and...
 */
void signal_handler(int signum, siginfo_t *info, void *ptr) {
	fprintf (stdout, "Received signal %d originating from PID %lu\n", signum, (unsigned long)info->si_pid);
	//register_write (uart_device_handle, 0x03, 0x00);

	if (stop_signal) {
		exit (-1);
	}

	stop_signal = 1;
}

void exit_handler(void) {
	//debug(1,"exit_handler():");
}

void main (int argc, char **argv) {

	int mode = 0;

	// Parse command line arguments. See usage() for details.
	char c;
	while ((c = getopt(argc, argv, "m:")) != -1) {
		switch(c) {

			case 'm':
				if (strcmp(optarg,"envelope")==0) {
					mode = MODE_ENVELOPE;
				} else if (strcmp(optarg,"iq")==0) {
					mode = MODE_IQ;
				} else {
					//error ("unrecognized output format '%s'\n",optarg);
					exit(-1);
				}
           		 	break;

		}
	}

	// Setup event handlers
	atexit(exit_handler);

	// Setup signal handler. Catching SIGPIPE allows for exit when 
	// piping to Wireshark for live packet feed.
	//signal(SIGPIPE, signal_handler);
	struct sigaction act;
	memset(&act, 0, sizeof(act));
	act.sa_sigaction = signal_handler;
	act.sa_flags = SA_SIGINFO;
	sigaction(SIGPIPE, &act, NULL);
	sigaction(SIGINT, &act, NULL);


	int device = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
	uart_set_interface_attribs (device, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)

	// Copy to global for exit handler
	//uart_device_handle = device;

	switch (mode) {
		case MODE_ENVELOPE:
		envelope_service(device);
		break;
		case MODE_IQ:
		iq_service(device);
		break;
	}

}




