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

// Max size of buffer needed to store one frame/sweep
// For IQ 16 bit x 2 x number of points
#define FRAME_BUF_SIZE 65536



// Signal handler sets this to 1 when signal received.
// Monitored by data dump loop to exit.
int stop_signal = 0;

void usage() {
	fprintf (stderr, "xm122 [-b] [-m mode] [-p profile] [-r start-of-range-mm] [-l length-of-range-mm]\n");
	fprintf (stderr," -b : increase baudrate from default 115200\n");
	fprintf (stderr," -m mode :envelope or iq\n");
	fprintf (stderr," -p profile : 1=highest res... 5=highest gain\n");
}

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

fprintf(stderr,"%d %d\n",address,value);

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

	// TODO: not gauranteed that all will be written in open write() call
	int bytes_written = 0;
	while (bytes_written < sizeof(register_write_frame) ) {
		bytes_written += write (device, &register_write_frame + bytes_written, sizeof(register_write_frame));
	}

	// Read the write register response back (10 bytes) and ignore
	uint8_t buf[10];
	int bytes_read = 0;
	while (bytes_read < 10) {
		bytes_read += read (device, buf, 10);
	}
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

	uint8_t buf[FRAME_BUF_SIZE];

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

	uint8_t buf[FRAME_BUF_SIZE];

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


/**
 * 
 * @param r start of raange (cm from sensor)
 * @param l length of range (cm starting at r cm from sensor)
 */
void envelope_service (int device, int r, int l) {
	// envelope service mode
	register_write (device, 0x02, 0x02);

	// range  
	register_write (device, 0x20, r);
	register_write (device, 0x21, l);

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
	fprintf (stderr, "Received signal %d originating from PID %lu\n", signum, (unsigned long)info->si_pid);
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

	int mode = -1;

	int baud_boost = 0;
	int profile = 2;

	// Parse command line arguments. See usage() for details.
	// declaration 'char c' does not work on Raspberry Pi. 
	// char seems to be redefined as uint8_t or something
	// -1 tests as 255 !!??!!
	//char c;
	int8_t c;
	int r=40,l=1000;
	while ((c = getopt(argc, argv, "bhl:m:p:r:")) != -1) { 
		switch(c) {
			case 'b':
				baud_boost = 1;
				break;
			case 'h':
				usage();
				exit(0);
			case 'm':
				if (strcmp(optarg,"envelope")==0) {
					mode = MODE_ENVELOPE;
				} else if (strcmp(optarg,"iq")==0) {
					mode = MODE_IQ;
				} else {
					fprintf (stderr,"unrecognized output format '%s'\n",optarg);
					exit(-1);
				}
           		 	break;
			case 'p':
				profile = atoi(optarg);
				if (profile<1 || profile>5) {
					fprintf (stderr,"invalid profile number: 1-5 allowed\n");
					exit(-1);
				}
				break;
			case 'r':
				r = atoi(optarg);
				break;
			case 'l':
				l = atoi(optarg);
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


	if (baud_boost) {
		register_write(device, 0x7, 921600 );
		uart_set_interface_attribs (device, B921600, 0); 
	}

	// Set profile register
	register_write(device, 0x28, profile);

	// Copy to global for exit handler
	//uart_device_handle = device;

	switch (mode) {
		case MODE_ENVELOPE:
		envelope_service(device,r,l);
		break;
		case MODE_IQ:
		iq_service(device);
		break;
		default:
		fprintf(stderr,"unknown mode\n");
		break;
	}

	// Clear UART buffer
	sleep(2);
	tcflush(device,TCIOFLUSH);

	if (baud_boost) {
                register_write(device, 0x7, 115200);
                uart_set_interface_attribs (device, B115200, 0);
	}
}




