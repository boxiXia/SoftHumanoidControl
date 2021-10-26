#ifndef __HOST_H
#define __HOST_H

#define MOTOR_NUM				12			// Number of ESCs
#define USB_UART_SPEED     		1000000		// Baudrate of the teeensy USB serial link
#define HOST_ERROR_FD         	-1			// Non existant file descriptor
#define HOST_ERROR_DEV        	-2			// Non existant serial device
#define HOST_ERROR_WRITE_SER  	-3			// Write error on serial
#define HOST_ERROR_BAD_PK_SZ  	-4			// Bad incoming packet size error

//#define  HOST_MODEMDEVICE		"usb-Teensyduino_USB_Serial_8784100-if00"
#define HOST_MODEMDEVICE    	"/dev/ttyACM0"	// Name of USB port

// Serial number of the teensy, check this number by using terminal, 
// input: cd /dev/serial/by-id/
// then : ls
#define HOST_DEV_SERIALNB		8784100			
#define HOST_DEV_SERIALLG		10				// Max length of a serial number
#define HOST_SERIAL_DEV_DIR		"/dev/serial/by-id/"
#define HOST_BAUDRATE       	B1000000		// Serial baudrate
#define DRIVE_RATIO 			8.0 		// Drive ratio of motor gear box
#define SEND_SIZE 5
#define STEP_SIZE  3
#define BUFFER_SIZE 15
static int ret;
static int fd;
#define BAUD 460800 // 115200 for JY61 ,9600 for others
float desired_pos[MOTOR_NUM] = {0}; // The deisred speed
float motor_mode;

// Teensy->host communication data structure
// sizeof(ESCPID_comm)=64 to match USB 1.0 buffer size
typedef struct {
  float    joint_pos[MOTOR_NUM];      // Motors rotation angle
  float    joint_vel[MOTOR_NUM];     // Motors rad/s
  float	   joint_cur[MOTOR_NUM];     // Motors torque
  float    acc[3];						  // Acceleration in X Y Z direction, m/s^2
  float    gyr[3];						  // Gyroscope in X Y Z direction, deg/s
  float    mag[3];             // Magnetometer in X Y Z, uT
  float    euler[3];
  float    foot_force[4];
  float    timestamps;
} Teensycomm_struct_t;

// Host->teensy communication data structure
// sizeof(RPi_comm)=64 to match USB 1.0 buffer size
typedef struct {
  float    comd[MOTOR_NUM];        		// Desired Speed, rad/s
  float    motor_mode[1];
} Jetson_comm_struct_t;

Teensycomm_struct_t teensy_comm;  // A data struct received from Teensy
Jetson_comm_struct_t jetson_comm; // A data struct sent to Teensy

int Host_fd = HOST_ERROR_FD; // Serial port file descriptor
char Host_devname[PATH_MAX] = ""; // Serial port devname used to get fd with open
struct termios Host_oldtio; // Backup of initial tty configuration

// Prototypes
char *Host_name_from_serial(uint32_t);
int   Host_get_fd(uint32_t);
int   Host_init_port(uint32_t);
void  Host_release_port(uint32_t);
int   Host_comm_update(uint32_t, Teensycomm_struct_t**);

//
// Get the device name from the device serial number
//
char *Host_name_from_serial(uint32_t serial_nb)
{
	DIR *d;
	struct dirent *dir;
	char serial_nb_char[HOST_DEV_SERIALLG];
	static char portname[PATH_MAX];

	// Convert serial number into string
	snprintf(serial_nb_char, HOST_DEV_SERIALLG, "%d", serial_nb);

	// Open directory where serial devices can be found
	d = opendir(HOST_SERIAL_DEV_DIR);

	// Look for a device name contining teensy serial number
	if (d)
	{
		// Scan each file in the directory
		while ((dir = readdir(d)) != NULL)
		{
			if (strstr(dir->d_name, serial_nb_char))
			{

				// A match is a device name containing the serial number
				snprintf(portname, PATH_MAX, "%s%s", HOST_SERIAL_DEV_DIR, dir->d_name);
				return portname;
			}
		}
		closedir(d);
	}
	return NULL;
}

//
// Get the file descriptor index which device name contains
// specified serial number.
// Returns -1 if no matching fd is found.
//
int Host_get_fd(uint32_t serial_nb)
{
	char serial_nb_char[HOST_DEV_SERIALLG];

	// Convert serial number into string
	snprintf(serial_nb_char, HOST_DEV_SERIALLG, "%d", serial_nb);

	if (Host_fd != HOST_ERROR_FD)
		if (strstr(Host_devname, serial_nb_char))
			return 0;

	return HOST_ERROR_FD;
}

//
// Initialize serial port
//
int Host_init_port(uint32_t serial_nb)
{
	struct termios newtio;
	int check_fd;
	char *portname;

	// Check if device plugged in
	portname = Host_name_from_serial(serial_nb);
	if (!portname)
		return HOST_ERROR_DEV;

	// Open device
	check_fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (check_fd < 0)
	{
		perror(portname);
		return HOST_ERROR_DEV;
	}

	// Store the fd and the corresponding devname
	Host_fd = check_fd;
	strncpy(Host_devname, portname, PATH_MAX);


	/* Save current port settings */
	tcgetattr(check_fd, &Host_oldtio);

	/* Define new settings */
	bzero(&newtio, sizeof(newtio));
	cfmakeraw(&newtio);

	newtio.c_cflag = HOST_BAUDRATE | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;

	/* Apply the settings */
	tcflush(check_fd, TCIFLUSH);
	tcsetattr(check_fd, TCSANOW, &newtio);

	return 0;
}

//
// Release serial port
//
void Host_release_port(uint32_t serial_nb)
{
	int fd_idx;

	// Get fd index from serial number
	fd_idx = Host_get_fd(serial_nb);

	if (fd_idx != HOST_ERROR_FD)
	{
		// Restore initial settings if needed
		tcsetattr(Host_fd, TCSANOW, &Host_oldtio);
		close(Host_fd);

		// Clear fd and corresponding devname
		Host_fd = HOST_ERROR_FD;
		strncpy(Host_devname, "", PATH_MAX);
	}
}

//
// Manage communication with the teensy connected to portname
//
int Host_comm_update(uint32_t serial_nb,

					 Teensycomm_struct_t **comm)
{

	int ret, res = 0;
	uint8_t *pt_in;

	// Get fd index
	Host_get_fd(serial_nb);

	// Update output data structue
	for (int i = 0; i < MOTOR_NUM; i++)
		jetson_comm.comd[i] = desired_pos[i];
	jetson_comm.motor_mode[0]=motor_mode;
	// Send output structure
	res = write(Host_fd, &jetson_comm, sizeof(jetson_comm));
	if (res < 0)
	{
		perror("write jetson_comm");
		return HOST_ERROR_WRITE_SER;
	}

	// Flush output buffer
	fsync(Host_fd);

	// Reset byte counter and magic number
	res = 0;
	pt_in = (uint8_t *)(&teensy_comm);

	do
	{
		ret = read(Host_fd, &pt_in[res], 1);

		// Data received
		if (ret > 0)
			res += ret;

		// Read error
		if (ret < 0)
			break;
	} while (res < sizeof(teensy_comm));

	// Check response size
	if (res != sizeof(teensy_comm))
	{
		fprintf(stderr, "Packet with bad size received.\n");

		// Flush input buffer
		while ((ret = read(Host_fd, pt_in, 1)))
			if (ret <= 0)
				break;

		return HOST_ERROR_BAD_PK_SZ;
	}

	// Return pointer to teensy_comm structure
	*comm = &teensy_comm;

	return 0;
}

inline int uart_open(int fd, const char *pathname) {
  fd = open(pathname, O_RDWR | O_NOCTTY);
  if (-1 == fd) {
    perror("Can't Open Serial Port");
    return (-1);
  } else
    printf("open %s success!\n", pathname);
  if (isatty(STDIN_FILENO) == 0)
    printf("standard input is not a terminal device\n");
  else
    printf("isatty success!\n");
  return fd;
}

int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
  struct termios newtio, oldtio; // save old setting for port
  if (tcgetattr(fd, &oldtio) !=
      0) { // get the parameters associated with the terminal
    perror("SetupSerial 1");
    printf("tcgetattr( fd,&oldtio) -> %d\n", tcgetattr(fd, &oldtio));
    return -1;
  }
  // erases the data in the n bytes of the memory starting at the location
  // pointed to by s, by writing zeros
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;
  switch (nBits) {
  case 7:
    newtio.c_cflag |= CS7;
    break;
  case 8:
    newtio.c_cflag |= CS8;
    break;
  }
  switch (nEvent) // parity check
  {
  case 'o':
  case 'O': // odd
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= (INPCK | ISTRIP);
    break;
  case 'e':
  case 'E': // even
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'n':
  case 'N': // none
    newtio.c_cflag &= ~PARENB;
    break;
  default:
    break;
  }

  /*set baud rate*/
  switch (nSpeed) {
  case 2400:
    cfsetispeed(&newtio, B2400);
    cfsetospeed(&newtio, B2400);
    break;
  case 4800:
    cfsetispeed(&newtio, B4800);
    cfsetospeed(&newtio, B4800);
    break;
  case 9600:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  case 115200:
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    break;
  case 460800:
    cfsetispeed(&newtio, B460800);
    cfsetospeed(&newtio, B460800);
    break;
  default:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  }
  // set stop bit
  if (nStop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (nStop == 2)
    newtio.c_cflag |= CSTOPB;
  // set wait time and smallest receving character
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  tcflush(fd, TCIFLUSH);

  // apply the setup
  if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
    perror("com set error");
    //   return -1;
    exit(EXIT_FAILURE);
  }
  printf("set done!\n");
  return 0;
}

int uart_close(int fd) {
  assert(fd);
  close(fd);
  // cleanup here
  return 0;
}
inline int send_data(int fd, char *send_buffer, int length) {
  // ssize_t write(int fildes, const void *buf, size_t nbyte);
  length = write(fd, send_buffer, length * sizeof(unsigned char));
  return length;
}
inline int recv_data(int fd, char *recv_buffer, int length) {
  // https://linux.die.net/man/3/read
  // ssize_t read(int fildes, void *buf, size_t nbyte);
  length = read(fd, recv_buffer, length);
  return length;
}

class Serial {
public:
  int fd;
  char r_buf[1024] = {0};
  Serial(const char *pathname) {
    fd = uart_open(fd, pathname);
  }

  int uartSet(int nSpeed, int nBits, char nEvent, int nStop) {
    return uart_set(fd, nSpeed, nBits, nEvent, nStop);
  }

  int send(char *send_buffer, int length) {
    return send_data(fd, send_buffer, length);
  }

  int recv(char *recv_buffer, int length) {
    return recv_data(fd, recv_buffer, length);
  }

  void close() {
    int ret = uart_close(fd);
    if (ret == -1) {
      fprintf(stderr, "uart_close error\n");
      exit(EXIT_FAILURE);
    }
  }

};

#endif
