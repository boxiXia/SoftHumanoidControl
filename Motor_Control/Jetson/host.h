#ifndef __HOST_H
#define __HOST_H

/*convert euler (roll,pitch,yaw) in rad to rotation matrix */
void eulerTomatrix(float& w, float& v, float& u,float rt_mat[3][3]){
	rt_mat[0][0]=cos(v)*cos(w);
	rt_mat[1][0]=sin(w)*cos(v);
	rt_mat[2][0]=-sin(v);
	rt_mat[0][1]=-sin(w)*cos(u)+cos(w)*sin(u)*sin(v);
	rt_mat[1][1]=cos(u)*cos(w)+sin(u)*sin(v)*sin(w);
	rt_mat[2][1]=cos(v)*sin(u);
	rt_mat[0][2]=sin(u)*sin(w)+cos(u)*sin(v)*cos(w);
	rt_mat[1][2]=-cos(w)*sin(u)+sin(w)*sin(v)*cos(u);
	rt_mat[2][2]=cos(u)*cos(v);
}

void matrix_dot(const float A[3][3], const float B[3][3], float C[3][3]){
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			float sum = 0;
			for (int m = 0; m < 3; m++){
				sum = sum + A[i][m] * B[m][j];
			}
			C[i][j] = sum;
		}
	}
}


/*--------------------------------------------------------*/
inline int uart_open(int fd, const char *pathname) {
  fd = open(pathname, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (-1 == fd) {
    printf("Can't Open Serial Port %s\n",pathname);
    exit (-1);
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
  case 'O': // odd parity
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= (INPCK | ISTRIP);
    break;
  case 'e':
  case 'E': // even parity
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'n':
  case 'N': // none parity
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
  case 500000:
    cfsetispeed(&newtio, B500000);
    cfsetospeed(&newtio, B500000);
    break;
  case 576000:
    cfsetispeed(&newtio, B576000);
    cfsetospeed(&newtio, B576000);
    break;
  case 921600:
    cfsetispeed(&newtio, B921600);
    cfsetospeed(&newtio, B921600);
    break;
  case 1000000:
    cfsetispeed(&newtio, B1000000);
    cfsetospeed(&newtio, B1000000);
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


template <class DataStruct>
int sendStruct(DataStruct data){
  int length = write(fd, &data, sizeof(DataStruct));
  // Flush output buffer
	fsync(fd);
  return length;
}

template <class DataStruct>
void recvStruct(DataStruct& data){
	int res = 0;
	int ret;
  // DataStruct data;  // A data struct received from Teensy
  uint8_t *pt_in = (uint8_t *)(&data);

  do{
    ret = read(fd, &pt_in[res], 1);
		// Data received
		if (ret > 0)
			res += ret;
		// Read error
		if (ret < 0)
			break;
	} while (res < sizeof(DataStruct));

  // res = read(fd, pt_in, sizeof(data));

	// Check response size
	if (res != sizeof(DataStruct))
	{
		fprintf(stderr, "Packet with bad size received.\n");
		// Flush input buffer
		while ((ret = read(fd, pt_in, 1)))
			if (ret <= 0)
				break;
		// return -4; //TODO...
	}

}

};

#endif
