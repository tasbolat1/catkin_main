// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <bitset>
#include <iostream>
#include <csignal>
#include <chrono>

using namespace std;

// Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
double durat_accum = 0.0;
int count_accum = 0;
int serial_port = open("/dev/ttyUSB1", O_RDONLY); //global variable by design, so signal handler can access it and close port when ctrl+C

void signalHandler( int signum ) {
   cout << "Interrupt signal (" << signum << ") received.\n";
   cout << "Average duration: " << durat_accum / count_accum;

   // cleanup and close up stuff here  
   // terminate program  
   close(serial_port);
   exit(signum);  
}

int main()
{
	signal(SIGINT, signalHandler); 

	// Create new termios struc, we call it 'tty' for convention
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	// Read in existing settings, and handle any error
	if(tcgetattr(serial_port, &tty) != 0) {
	    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be 115200
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}

	// Allocate memory for read buffer, set size according to your needs
	char read_buf [1];
	memset(&read_buf, '\0', sizeof(read_buf));

	// Read bytes. The behaviour of read() (e.g. does it block?,
	// how long does it block for?) depends on the configuration
	// settings above, specifically VMIN and VTIME
	while (true){
		auto start = std::chrono::system_clock::now();
		
		int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end-start;
		//std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
		if (elapsed_seconds.count() < 0.05){
			durat_accum += elapsed_seconds.count();
			count_accum += 1;
		}

		// n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
		if (num_bytes < 0) {
		    printf("Error reading: %s", strerror(errno));
		}
		if (num_bytes > 0) {
			for (auto val : read_buf) {
				std::bitset<8> bit8(val);
				std::cout << bit8 << " ";
			}
			printf("\n");
		}
		memset(&read_buf, '\0', sizeof(read_buf));
	}	
}

