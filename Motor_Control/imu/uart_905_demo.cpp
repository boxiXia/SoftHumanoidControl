#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

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






/*-------------------------------------------------------------------------------------------*/
static int ret;
static int fd;
#define BAUD 460800 // 115200 for JY61 ,9600 for others

float a[3], w[3], Angle[3], h[3];
void ParseData(char chr) {
  static char chrBuf[100];
  static unsigned char chrCnt = 0;
  signed short sData[4];
  unsigned char i;
  char cTemp = 0;
  time_t now;
  chrBuf[chrCnt++] = chr;
  if (chrCnt < 11)
    return;
  for (i = 0; i < 10; i++)
    cTemp += chrBuf[i];
  if ((chrBuf[0] != 0x55) || ((chrBuf[1] & 0x50) != 0x50) ||
      (cTemp != chrBuf[10])) {
    printf("Error:%x %x\r\n", chrBuf[0], chrBuf[1]);
    memcpy(&chrBuf[0], &chrBuf[1], 10);
    chrCnt--;
    return;
  }

  memcpy(&sData[0], &chrBuf[2], 8);
  switch (chrBuf[1]) {
  case 0x51:
    for (i = 0; i < 3; i++)
      a[i] = (float)sData[i] / 32768.0 * 16.0;
    time(&now);
    printf("\r\nT:%s a:%6.3f %6.3f %6.3f ", asctime(localtime(&now)), a[0],
           a[1], a[2]);
    break;
  case 0x52:
    for (i = 0; i < 3; i++)
      w[i] = (float)sData[i] / 32768.0 * 2000.0;
    printf("w:%7.3f %7.3f %7.3f ", w[0], w[1], w[2]);
    break;
  case 0x53:
    for (i = 0; i < 3; i++)
      Angle[i] = (float)sData[i] / 32768.0 * 180.0;
    printf("A:%7.3f %7.3f %7.3f ", Angle[0], Angle[1], Angle[2]);
    break;
  case 0x54:
    for (i = 0; i < 3; i++)
      h[i] = (float)sData[i];
    printf("h:%4.0f %4.0f %4.0f ", h[0], h[1], h[2]);

    break;
  }
  chrCnt = 0;
}

int main() {
  Serial imu("/dev/ttyUSB0");
  imu.uartSet(BAUD, 8, 'N', 1);
  char r_buf[1024];
  while (1) {
    ret = imu.recv(r_buf, 44);
    for (int i = 0; i < ret; i++) {
      ParseData(r_buf[i]);
      usleep(1000);
    }
  }
  imu.close();
  return 0;
}

// int main(void) {
//   char r_buf[1024];
//   bzero(r_buf, 1024);

//   fd = uart_open(fd, "/dev/ttyUSB0"); /*串口号/dev/ttySn,USB口号/dev/ttyUSBn
//   */ if (fd == -1) {
//     fprintf(stderr, "uart_open error\n");
//     exit(EXIT_FAILURE);
//   }

//   if (uart_set(fd, BAUD, 8, 'N', 1) == -1) {
//     fprintf(stderr, "uart set failed!\n");
//     exit(EXIT_FAILURE);
//   }

//   //   FILE * fp;
//   //   fp = fopen("Record.txt", "w");
//   while (1) {
//     ret = recv_data(fd, r_buf, 44);
//     if (ret == -1) {
//       fprintf(stderr, "uart read failed!\n");
//       exit(EXIT_FAILURE);
//     }
//     for (int i = 0; i < ret; i++) {
//       //   fprintf(fp, "%2X ", r_buf[i]);
//       ParseData(r_buf[i]);
//     }
//     usleep(1000);
//   }

//   ret = uart_close(fd);
//   if (ret == -1) {
//     fprintf(stderr, "uart_close error\n");
//     exit(EXIT_FAILURE);
//   }

//   exit(EXIT_SUCCESS);
// }