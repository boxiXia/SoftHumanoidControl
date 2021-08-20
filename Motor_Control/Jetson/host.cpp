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
#include "host.h"

// using namespace std;

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
	float timestamps;
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
	MSGPACK_DEFINE_ARRAY(timestamps, joint_pos, joint_vel, joint_cur, orientation,  gyr, acc, euler,joint_posraw);
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
	for (int i = 0; i < MOTOR_NUM; ++i)
		desired_pos[i] = 0.0;
	motor_mode=0; // 0:position control
	Teensycomm_struct_t *comm;
	int ret;

	// Initialize serial port
	if (Host_init_port(HOST_DEV_SERIALNB)){
		fprintf(stderr, "Error initializing serial port.\n");
		exit(-1);
	}
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
	//UdpDataSend msg_send;			// For holding the sent class
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
		// Serial exchange with teensy
		if ((ret = Host_comm_update(HOST_DEV_SERIALNB, &comm)))
		{
			fprintf(stderr, "Error %d in Host_comm_update.\n", ret);
			break;
		}
		// After receiving the data from Teensy, save it into class msg_to_pc
		
		data_send.timestamps = comm->timestamps;
		MsgFromPC recv = udp_receiver.msg_queue.front();
		for (int j = 0; j < MOTOR_NUM; ++j)
		{
			// Angle of motors:
			data_send.joint_posraw[j] = comm->joint_pos[j];
			// Speed of motors:
			data_send.joint_vel[j] = comm->joint_vel[j];
			// Angle of motors:
			data_send.joint_pos[j*2] = std::cos(comm->joint_pos[j]);
			data_send.joint_pos[j*2+1] = std::sin(comm->joint_pos[j]);
			// Current of motors:
			data_send.joint_cur[j] = comm->joint_cur[j];
		}

		for (int i = 0; i < 3; ++i){
			// msg_to_pc.mag[i] = comm->mag[i];
			data_send.acc[i] = comm->acc[i];
			data_send.gyr[i] = comm->gyr[i];
			data_send.euler[i] = comm->euler[i];
			//printf("%f\t", msg_to_pc.euler[i]);
		}
		//printf("\n");
		eulerTomatrix(comm->euler[0],comm->euler[1],comm->euler[2],imu_transform);

		matrix_dot(R, R_imuinverse,Rw_r1);
		matrix_dot(Rw_r1,imu_transform,Rw_r);

		float Rall[6]={Rw_r[0][0],Rw_r[1][0],Rw_r[2][0],Rw_r[0][1],Rw_r[1][1],Rw_r[2][1]};
		for (int i = 0; i < 6; ++i){
			data_send.orientation[i] =Rall[i];
		}
		//std::cout <<Rw_r<< '\n';
		//printf("%f%f%f\t", Rw_r1[0][0],Rw_r1[0][1],Rw_r1[0][2]);

		//printf("%f\t", data_send.euler[0]);
		
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
	
	Host_release_port(HOST_DEV_SERIALNB);

	udp_receiver.RUNNING = false;
	if (recv_thread.joinable()){recv_thread.join();}

	return 0;
}
