/************************************************************
This is a test to check the viability of termios as an option
for writing driver for the PNI TRAX IMU. This a sort of proof
of concept attempt at utilizing the library. Termios is a st-
andard library and uses struct termios objects to access lin-
ux dev/tty/ ports. This test is don with a KL25Z running a s-
imple logger system across the UART. The logger asynchronous-
ly logs any information the system finds relevant with respe-
ct to the current state of the device.
************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>

#define POLL_RESP_LENGTH	(0x19)


struct termios *configure;
void tty_config(struct termios *con, int descriptor);

char *device = "/dev/ttyACM0";
int fd;
uint8_t len = POLL_RESP_LENGTH, c;

void tty_config(struct termios *con, int descriptor)
{
	tcgetattr(descriptor, con);
	con->c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	con->c_oflag = 0;
	con->c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	con->c_cc[VMIN] = 1;
	con->c_cc[VTIME] = 0;
	if(cfsetispeed(con, B9600) || cfsetospeed(con, B9600))
	{
		perror("ERROR in baud set\n");
	}
	if(tcsetattr(descriptor, TCSAFLUSH, con) < 0)
	{
		perror("ERROR in set attr\n");
	}
}


void poll_for_data()
{
	while(1)
	{
		if(read(fd, &c, 1) > 0)
		{
			printf("%c", c);
			fflush(stdout);
			len--;
		}
	}
}


int main()
{
	fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(fd == -1)
	{
		perror("ERROR opening file descriptor\n");
	}

	configure = (struct termios*)malloc(sizeof(struct termios));
	tty_config(configure, fd);
	poll_for_data();
	close(fd);
	return(0);
}
