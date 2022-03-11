#ifndef __HOST_H
#define __HOST_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <dirent.h>
#include <iostream>
#include <linux/input.h>
#include <msgpack.hpp>
#include <thread>
#include <deque>
#include <chrono>
#include "host.h"
#include <fstream>
/*
  z-y-x sequence,i.e first rotate about local z, then rotate about local y, 
  finally rotate about local x, equivalent to (python):
  from scipy.spatial.transform import Rotation as R
  R.from_euler('xyz',(rx,ry,rz),degrees = True).as_matrix()
*/
template <typename T1,typename T2>
void eulerToRotationMatrix(const T1&rx,const T1& ry,const T1& rz,T2 m[3][3]){
    T2 c0 = cos(rx);
    T2 c1 = cos(ry);
    T2 c2 = cos(rz);
    T2 s0 = sin(rx);
    T2 s1 = sin(ry);
    T2 s2 = sin(rz);
    m[0][0] = c1*c2; m[0][1] = s1*c2*s0-c0*s2; m[0][2] = s0*s2+c0*s1*c2;
    m[1][0] = c1*s2; m[1][1] = s0*s1*s2+c0*c2; m[1][2] = c0*s1*s2-s0*c2;
    m[2][0] = -s1;   m[2][1] = s0*c1;          m[2][2] = c0*c1;
}

/*q = qx*i+qy*j+qz*k+qw*/
template <typename T1,typename T2>
void quaternionToRotationMatrix(const T1& qw,const T1& qx,const T1& qy,const T1& qz,T2 m[3][3]){
    T2 xx = qx*qx;
    T2 yy = qy*qy;
    T2 zz = qz*qz;
    T2 ww = qw*qw;
    T2 xy = qx*qy;
    T2 xz = qx*qz;
    T2 xw = qx*qw;
    T2 yz = qy*qz;
    T2 yw = qy*qw;
    T2 zw = qz*qw;
    T2 s2 = 2./(xx + yy + zz + ww);

    m[0][0] = 1-s2*(yy+zz); m[0][1] = s2*(xy-zw);   m[0][2] = s2*(xz+yw);
    m[1][0] = s2*(xy+zw);   m[1][1] = 1-s2*(xx+zz); m[1][2] = s2*(yz-xw);
    m[2][0] = s2*(xz-yw);   m[2][1] = s2*(yz+xw);   m[2][2] = 1-s2*(xx+yy);
}

template <typename T>
void matrixDot(const T A[3][3], const T B[3][3], T C[3][3]){
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			T sum = 0;
			for (int m = 0; m < 3; m++){
				sum += A[i][m] * B[m][j];
			}
			C[i][j] = sum;
		}
	}
}

void matrixDotvector(double A[3][3], float B[3], float C[3]){
	for (int i = 0; i < 3; i++){
    double sum = 0;
		for (int j = 0; j < 3; j++){
				sum += A[i][j] * B[j];
		}
    C[i] = sum;
	}
}


void findInverse(double minv[3][3], double m[3][3]){
  double invdet = 1/(m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
             m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
             m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]));
  minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
  minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
  minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
  minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
  minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
  minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
  minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
  minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
  minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;
}

template <typename T>
void printMatrix(T m[3][3]){
  std::cout<<"\n";
	std::cout<<"[["<<m[0][0]<<","<<m[0][1]<<","<<m[0][2]<<"],\n";
	std::cout<<" ["<<m[1][0]<<","<<m[1][1]<<","<<m[1][2]<<"],\n";
	std::cout<<" ["<<m[2][0]<<","<<m[2][1]<<","<<m[2][2]<<"]]\n";
	std::cout<<"\n";
}


void readMatrix(double A[3][3],std::string path){
  std:: ifstream f;
  f.open(path);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      f >> A[i][j];
  f.close();
}

void writeMatrix(double A[3][3], std::string path){
  std:: ofstream f;
  f.open(path);
  printf("writing matrix\n");
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
    f << A[i][j];
    f << " ";
    }
  }
  f.close();
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
	uint res = 0;
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
