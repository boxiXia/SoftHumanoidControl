/*****************host.cpp******************/
// compile:
// sudo g++ -pthread host.cpp
// autorun: edit etc/rc.local:
//			add: path_to_a.out &
// run:
// ./a.out
/*******************************************/
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
#include <cmath>
#include <chrono>
#include "host.h"

constexpr int JETSON_PORT = 32004;
constexpr int PC_PORT = 32005;
constexpr char PC_IP_ADDR[] = "192.168.137.1";
// constexpr char PC_IP_ADDR[] = "192.168.1.109";
constexpr int UDP_BUFFER_SIZE = 1280;

// orientation calculation
float R_imuinverse[3][3] = 
{{0.0466428 , -0.99876216,  0.01727997},
 {0.99714982,  0.0455266 , -0.06016285},
 {0.05930168,  0.02003687,  0.998039  }};

float R[3][3] = 
{{ 0, 0,-1},
 { 0, 1, 0},
 { 1, 0, 0}};

float imu_transform[3][3] = {0};
float robot_transform[3][3] = {0};
float Rw_r[3][3] = {0};
float Rw_r1[3][3] = {0};
float Rw_r2[3][3] = {0};
float a[3], w[3], Angle[3], h[3];
// foot sensor ............
float c3_func[3] = {-0.4079, 0.0269, 2.9743}; //pin16:left_forefoot
float g3_func[3] = {-0.4278, 0.0167, 3.0961}; //pin17:left_backheel
float e1_func[3] = {-0.3057, 0.0167, 3.1222}; //pin20:right_forefoot
float a3_func[3] = {-0.3781, 0.0197, 3.0990}; //pin21:right_backheel

int avg_size = 10; // number of analog readings to average
float R_0 = 20000.0; // known resistor value in [ohms]
float Vcc = 5.0; //supply voltage
float leftfoot_mass = 0.0; //the mass on left foot
float rightfoot_mass = 0.0; //the mass on right foot
//...............
// Receiving the desired speed data by UDP and hold it in this class
class MsgFromPC
{
public:
	float robot_command[MOTOR_NUM];
	float motor_mode;
	MSGPACK_DEFINE_ARRAY(robot_command,motor_mode);
};


// unit messge to be sent to pc

class DataSend
{
public:
	// float timestamps;
	unsigned long timestamps;

	float joint_pos[MOTOR_NUM*2]; // Rotation angle, unit degree
	float joint_vel[MOTOR_NUM]; // Rotation speed, unit rad/s
	float joint_cur[MOTOR_NUM];
	float joint_posraw[MOTOR_NUM]; // Rotation current, unit A
	//float actuation[MOTOR_NUM]; 
	float acc[3];				// Acceleration of IMU, unit m/s^2
	float gyr[3];				// Gyroscope, unit deg/s
	float mag[3];
	float euler[3];
	float orientation[6];
	float foot_force[2];
	MSGPACK_DEFINE_ARRAY(timestamps, joint_pos, joint_vel, joint_cur, orientation,  gyr, acc, euler,foot_force,joint_posraw);
};

class MsgToPC
{
public:
	int header = 114514;
	DataSend data [SEND_SIZE];
	MSGPACK_DEFINE_ARRAY(header, data);
};
// set a object for sending UDP through msgpack
MsgToPC msg_to_pc;
DataSend data_send;
std::deque <DataSend> send_buf;

//foot sensor...............
float convert_4voltages_to_mass(float v1,float v2,float v3,float v4)  //v1:16; v2:17; v3:20; v4:21
{
    //Average 10 Voltage readings to sum_val 
  float val_c3 = v1;
  float val_g3 = v2;
  float val_e1 = v3;
  float val_a3 = v4;

  //R_FSR
  float R_FSR_c3;
  float R_FSR_g3;
  float R_FSR_e1;
  float R_FSR_a3;

  //Mass
  float Mass_c3;
  float Mass_g3;
  float Mass_e1;
  float Mass_a3;

  //get the R_FSR for each fsr
  R_FSR_c3 = (R_0/1000.0 * val_c3)/(Vcc - val_c3);
  R_FSR_g3 = (R_0/1000.0 * val_g3)/(Vcc - val_g3);
  R_FSR_e1 = (R_0/1000.0 * val_e1)/(Vcc - val_e1);
  R_FSR_a3 = (R_0/1000.0 * val_a3)/(Vcc - val_a3);
  
  Mass_c3 = pow(10, c3_func[0] * log10(R_FSR_c3) + c3_func[1]/R_FSR_c3 + c3_func[2]);
  Mass_g3 = pow(10, g3_func[0] * log10(R_FSR_g3) + g3_func[1]/R_FSR_g3 + g3_func[2]);
  Mass_e1 = pow(10, e1_func[0] * log10(R_FSR_e1) + e1_func[1]/R_FSR_e1 + e1_func[2]);
  Mass_a3 = pow(10, a3_func[0] * log10(R_FSR_a3) + a3_func[1]/R_FSR_a3 + a3_func[2]);

  //divided by 1000, from g to kg
  leftfoot_mass = (Mass_c3+Mass_a3)/1000;
  rightfoot_mass = (Mass_g3+Mass_e1)/1000;
  return leftfoot_mass, rightfoot_mass;
}
//.....................


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
    //printf("\r\nT:%s a:%6.3f %6.3f %6.3f ", asctime(localtime(&now)), a[0],
           //a[1], a[2]);
    break;
  case 0x52:
    for (i = 0; i < 3; i++)
      w[i] = (float)sData[i] / 32768.0 * 2000.0;
    //printf("w:%7.3f %7.3f %7.3f ", w[0], w[1], w[2]);
    break;
  case 0x53:
    for (i = 0; i < 3; i++)
      Angle[i] = (float)sData[i] / 32768.0 * 180.0;
    //printf("A:%7.3f %7.3f %7.3f ", Angle[0], Angle[1], Angle[2]);
    break;
  case 0x54:
    for (i = 0; i < 3; i++)
      h[i] = (float)sData[i];
    //printf("h:%4.0f %4.0f %4.0f ", h[0], h[1], h[2]);

    break;
  }
  chrCnt = 0;
}
void get_imu_data(Serial imu){
	char r_buf[1024];
	ret = imu.recv(r_buf, 44);
			for (int i = 0; i < ret; i++) {
			ParseData(r_buf[i]);
			//usleep(1000);
			}
}


/*convert euler (roll,pitch,yaw) to rotation matrix */
void eulerTomatrix(float& r, float& p, float& y,float rt_mat[3][3]){
	rt_mat[0][0]=cos(r)*cos(p);
	rt_mat[1][0]=sin(r)*cos(p);
	rt_mat[2][0]=-sin(p);
	rt_mat[0][1]=-sin(r)*cos(y)+cos(r)*sin(p)*sin(y);
	rt_mat[1][1]=cos(r)*cos(y)+sin(r)*sin(p)*sin(y);
	rt_mat[2][1]=cos(p)*sin(y);
	rt_mat[0][2]=sin(r)*sin(y)+cos(r)*sin(p)*cos(y);
	rt_mat[1][2]=-cos(r)*sin(y)+sin(r)*sin(p)*cos(y);
	rt_mat[2][2]=cos(p)*cos(y);
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
class UdpReceiver{
public:
	bool RUNNING = true; // indicating whether the main loop is running
	std::deque<MsgFromPC> msg_queue;
	/* udp receiver thread */
	void run()
	{
		/********************UDP_Receiving_Initializing********************/
		socklen_t fromlen;
		struct sockaddr_in server_recv;
		struct sockaddr_in hold_recv;

		int sock_recv = socket(AF_INET, SOCK_DGRAM, 0);

		int sock_length_recv = sizeof(server_recv);
		bzero(&server_recv, sock_length_recv);

		server_recv.sin_family = AF_INET;
		server_recv.sin_addr.s_addr = INADDR_ANY;
		server_recv.sin_port = htons(JETSON_PORT); // Setting port of this program
		bind(sock_recv, (struct sockaddr *)&server_recv, sock_length_recv);
		fromlen = sizeof(struct sockaddr_in);
		MsgFromPC recv;						// For holding the received class
		char buf_UDP_recv[UDP_BUFFER_SIZE]; // for holding UDP data
		/******************************************************************/
		while (RUNNING)
		{
			int datalength = recvfrom(sock_recv, buf_UDP_recv, UDP_BUFFER_SIZE, 0, (struct sockaddr *)&hold_recv, &fromlen);
			msgpack::object_handle oh = msgpack::unpack(buf_UDP_recv, datalength);
			msgpack::object obj = oh.get();
			obj.convert(recv);
			msg_queue.push_front(recv);
			while(msg_queue.size()>1){
				msg_queue.pop_back();
			}
		}
	}
};

int main()
{
	// Initialize corresponding data structure
	for (int i = 0; i < MOTOR_NUM; ++i)
		desired_pos[i] = 0.0;
	motor_mode=0; // 0:position control


	Teensycomm_struct_t comm;
	int ret;


	UdpReceiver udp_receiver;
	std::thread recv_thread(&UdpReceiver::run,&udp_receiver); // udp receiving in a separate thread
	printf("\ninitialized\n");

	/*********************UDP_Sending_Initializing*********************/
	struct sockaddr_in client_send; // local sending socket

	int sock_send = socket(AF_INET, SOCK_DGRAM, 0);

	int sock_length_send = sizeof(client_send);
	bzero(&client_send, sock_length_send);

	client_send.sin_family = AF_INET;
	client_send.sin_port = htons(PC_PORT);				   // Port of aim program
	inet_pton(AF_INET, PC_IP_ADDR, &client_send.sin_addr); // Address
	bind(sock_send, (struct sockaddr *)&client_send, sock_length_send);


	/*********************Serial for IMU *********************/
	Serial imu {"/dev/ttyUSB0"};
	//        baudrate,...
  	imu.uartSet(460800, 8, 'N', 1);
  	char r_buf[1024];

	/********************* Serial for teensy****************************/
	Serial teensy_serial {"/dev/ttyACM0"};
	teensy_serial.uartSet(1000000, 8, 'N', 1);



	/******************************************************************/
	while (1){
		if(udp_receiver.msg_queue.size()>0){
			MsgFromPC recv = udp_receiver.msg_queue.front();
			for (int i = 0; i < MOTOR_NUM; i++){
				desired_pos[i] = recv.robot_command[i];
			}
			motor_mode=recv.motor_mode;
		}
		//printf("%f\t", desired_pos[0]);


		// Update output data structue
		for (int i = 0; i < MOTOR_NUM; i++)
			jetson_comm.comd[i] = desired_pos[i];
		jetson_comm.motor_mode[0]=motor_mode;
		// send to teensy
		teensy_serial.sendStruct(jetson_comm);
		// receive from teensy
		
		teensy_serial.recvStruct(comm);

		// Teensycomm_struct_t comm_ = teensy_serial.recvStruct<Teensycomm_struct_t>();
		// comm = &comm_;

		get_imu_data(imu);//TODO.....

		// data_send.timestamps = comm->timestamps;

		unsigned long milliseconds_since_epoch = 
		std::chrono::duration_cast<std::chrono::microseconds>
			(std::chrono::system_clock::now().time_since_epoch()).count();
		data_send.timestamps = milliseconds_since_epoch;

		MsgFromPC recv = udp_receiver.msg_queue.front();
		for (int j = 0; j < MOTOR_NUM; ++j)
		{
			// Angle of motors:
			data_send.joint_posraw[j] = comm.joint_pos[j];
			// Speed of motors:
			data_send.joint_vel[j] = comm.joint_vel[j];
			// Angle of motors:
			data_send.joint_pos[j*2] = std::cos(comm.joint_pos[j]);
			data_send.joint_pos[j*2+1] = std::sin(comm.joint_pos[j]);
			// Current of motors:
			data_send.joint_cur[j] = comm.joint_cur[j];
		}
		//std::cout<<comm->joint_cur[1]<<" "<<comm->joint_cur[1]<< " "<<comm->joint_cur[2]<<'\n';
		for (int i = 0; i < 3; ++i){
			// msg_to_pc.mag[i] = comm->mag[i];
			data_send.acc[i] = a[i];
			data_send.gyr[i] = w[i];
			data_send.euler[i] = Angle[i];
		}
		// std::cout<<data_send.joint_cur[0]<<" "<<data_send.joint_cur[1]<< " "<<data_send.joint_cur[2]<<'\n';
		
		//std::cout<<Angle[0]<<" "<<Angle[1]<<" "<<Angle[2]<<'\n';
		//printf("\n");
		eulerTomatrix(Angle[0],Angle[1],Angle[2],imu_transform);

		matrix_dot(R, R_imuinverse,Rw_r1);
		matrix_dot(Rw_r1,imu_transform,Rw_r);

		float Rall[6]={Rw_r[0][0],Rw_r[1][0],Rw_r[2][0],Rw_r[0][1],Rw_r[1][1],Rw_r[2][1]};
		for (int i = 0; i < 6; ++i){
			data_send.orientation[i] =Rall[i];
		}
		// std::cout<<Rw_r[0][0]<<" "<<Rw_r[0][1]<<" "<<Rw_r[0][2]<<'\n';
		// std::cout<<Rw_r[1][0]<<" "<<Rw_r[1][1]<<" "<<Rw_r[1][2]<<'\n';
		// std::cout<<Rw_r[2][0]<<" "<<Rw_r[2][1]<<" "<<Rw_r[2][2]<<'\n';

		// Foot sensor data process............
		leftfoot_mass, rightfoot_mass=convert_4voltages_to_mass(comm.foot_force[0],comm.foot_force[2],comm.foot_force[1],comm.foot_force[3]);
		// on/off contact switch
		data_send.foot_force[0]= leftfoot_mass>0.5? 1:0;
		data_send.foot_force[1]= rightfoot_mass>0.5? 1:0;


		// data_send.foot_force[0]=leftfoot_mass;
		// data_send.foot_force[1]=rightfoot_mass;
		//std::cout<<comm->foot_force[0]<<" "<<comm->foot_force[2]<<" "<<comm->foot_force[1]<<" "<<comm->foot_force[3]<<'\n';
		//std::cout<<leftfoot_mass<<" "<<rightfoot_mass<<'\n';
		//.................

		
		send_buf.push_front(data_send);

		while (send_buf.size()>BUFFER_SIZE){send_buf.pop_back();}
		
		if (send_buf.size()==BUFFER_SIZE){
			for (int i = 0; i < SEND_SIZE; i++){
				msg_to_pc.data[i] = send_buf[i*STEP_SIZE];
			}
			
		std::stringstream send_stream;
		msgpack::pack(send_stream, msg_to_pc);
		std::string const &data = send_stream.str();
		sendto(sock_send, data.c_str(), data.size(), 0, (struct sockaddr *)&client_send, sizeof(client_send));
		}
	}
	

	udp_receiver.RUNNING = false;
	if (recv_thread.joinable()){recv_thread.join();}

	imu.close();
	teensy_serial.close();

	return 0;
}
