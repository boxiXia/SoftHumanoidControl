/*****************host.cpp******************/
/*
To compile:
	sudo g++ -Wall -pthread host.cpp -o robot_control.o
To setup autorun, edit /etc/rc.local:
# bash:
	sudo nano /etc/rc.local
add following lines:h
	path_to_the_robot_control.o &
also make sure that the first line of the rc.local looks like:
	#!/bin/sh -e
you can check with this bash command
	head -n1 /etc/rc.local
make sure that rc.local is executable,to check:
	ls -l /etc/rc.local
You should see:
	 -rwxr-xr-x 1 root root 419 2010-08-27 11:26 /etc/rc.local
if not, run this line:
	sudo chmod +x /etc/rc.local 
To run manually:
	./robot_control.o

To connect the serial IMU, refer to:https://www.jetsonhacks.com/2019/10/10/jetson-nano-uart/
*/
/*******************************************/
#include "host.h"


constexpr int JETSON_PORT = 32004;
constexpr int PC_PORT = 32005;
constexpr char PC_IP_ADDR[] = "192.168.137.1";
// constexpr char PC_IP_ADDR[] = "192.168.1.109";
constexpr int UDP_BUFFER_SIZE = 1280;

constexpr int SEND_SIZE = 5;
constexpr int STEP_SIZE = 3;
constexpr int BUFFER_SIZE = SEND_SIZE * STEP_SIZE;

static int ret;

constexpr int MOTOR_NUM = 12;		// Number of ESCs
float desired_pos[MOTOR_NUM] = {0}; // The deisred speed

// Teensy->host communication data structure
// sizeof(ESCPID_comm)=64 to match USB 1.0 buffer size
typedef struct
{
	float joint_pos[MOTOR_NUM]; // Motors rotation angle
	float joint_vel[MOTOR_NUM]; // Motors rad/s
	float joint_cur[MOTOR_NUM]; // Motors torque
	int16_t foot_force[4];
	float timestamps;
} Teensycomm_struct_t;

// Host->teensy communication data structure
// sizeof(RPi_comm)=64 to match USB 1.0 buffer size
typedef struct
{
	u_int16_t header;
	float comd[MOTOR_NUM]; // Desired pos, rad
} Jetson_comm_struct_t;


//...............
// Receiving the desired speed data by UDP and hold it in this class
class MsgFromPC
{
public:
	u_int16_t header;
	float robot_command[MOTOR_NUM];
	MSGPACK_DEFINE_ARRAY(header,robot_command);
};

// unit messge to be sent to pc

class DataSend
{
public:
	// float timestamps;
	int header = 114514;
	unsigned long timestamps;
	float joint_pos[MOTOR_NUM * 2]; // Rotation angle, unit rad
	float joint_vel[MOTOR_NUM];		// Rotation speed, unit rad/s
	float joint_cur[MOTOR_NUM];
	float orientation[6];
	float acc[3]; // Acceleration of IMU, unit m/s^2
	float angular_vel[3]; // Gyroscope, unit rad/s
	// float euler[3];
	float foot_force[2];
	MSGPACK_DEFINE_ARRAY(
		header,
		timestamps,   // 0
		joint_pos,    // 1
		joint_vel,    // 2
		joint_cur,    // 3
		orientation,  // 4
		angular_vel,  // 5
		acc,          // 6
		foot_force    // 7
		); 
};




// foot sensor ............

constexpr float a3_func[3] = {-0.3781, 0.0197, 3.0990}; //left_forefoot 
constexpr float c3_func[3] = {-0.4079, 0.0269, 2.9743}; //left_backheel
constexpr float e1_func[3] = {-0.3057, 0.0272, 3.1222}; //right_forefoot
constexpr float g3_func[3] = {-0.4278, 0.0167, 3.0961}; //right_backheel


#define SENSOR_PIN_NUM 4
 
constexpr float fsr_coeff[SENSOR_PIN_NUM][3] = {
	{-0.3781, 0.0197, 3.0990}, //a3, left_forefoot 
	{-0.4079, 0.0269, 2.9743}, //c3, left_backheel
	{-0.3057, 0.0272, 3.1222}, //e1, right_forefoot
	{-0.4278, 0.0167, 3.0961} //g3, right_backheel
};


float R_FSR[SENSOR_PIN_NUM];
float measured_mass[SENSOR_PIN_NUM];
constexpr float R_0 = 7.44;			// known resistor value in [Kohms]
float leftfoot_mass;	//the mass on left foot
float rightfoot_mass; //the mass on right foot
//foot sensor...............
void convert_4voltages_to_mass(int16_t v[])
{

	for(int i=0;i<SENSOR_PIN_NUM;++i){
		R_FSR[i] = R_0 * v[i] / (1023 - v[i]);
		measured_mass[i] = pow(10, fsr_coeff[i][0] * log10(R_FSR[i]) + fsr_coeff[i][1] / R_FSR[i] + fsr_coeff[i][2]);
	}
	leftfoot_mass = (measured_mass[0] + measured_mass[1]) / 1000;
	rightfoot_mass = (measured_mass[2] + measured_mass[3]) / 1000;
}



class IMU
{
public:

	
	
	// absolute world coordinate:w0
	// a fixed world coordinate we are interested in: w1
	// robot coordinate: rob
	// imu coordinate: imu
	// imu measures the transform of imu coordinate relative 
	// to the abs world coordinate, i.e. r_w0_imu
	// we want to find the robot transform relative to the fixed
	// coordinate w1, i.e. r_w1_rob
	// r_w1_imu = r_w1_w0 @ r_w0_imu
	// calibration: 
	// to find out r_w1_w0, we calibrate the imu at certain pose,
	// suppose we know imu transform relative to coordate w1 (r_w1_imu)
	// imu measures its transform relative to coordinate w0, (r_w0_imu)
	// r_w1_w0 = r_w1_imu @ (r_w0_imu^-1)
	// robot transform relative to imu is known: r_imu_rob
	// final matrix: r_w1_rob =  r_w1_imu @ r_imu_rob
	//                        =  r_w1_w0 @ r_w0_imu @ r_imu_rob

	// imu transform relative to the w1 coordiate at calibartion
	double r_w1_imu_at_calibartion[3][3] = {
		{1,0,0},
		{0,1,0},
		{0,0,1}
		}; 

	double r_w1_imu[3][3];
	double r_imu_w1[3][3];

	// transform from imu to robot
	double r_imu_rob[3][3] = { 
		{0, 0, 1},
		{0, -1, 0},
		{1, 0, 0}};

	// transform from w1 to w0
	double r_w1_w0[3][3]; 
	
	// transform from w1 to robot (this is what we are insterested in)
	double r_w1_rob[3][3];

	double r_w0_imu[3][3];

private:
	float _acc[3];
	float _angular_vel[3];
public:
	float acc[3]; // acceleration expressed at w1, gravity subtracted

	float angular_vel[3];
	float euler_angle[3];
	float magnetic_field[3];
	float quat[4];

	// Serial serial{"/dev/ttyUSB0"};
	Serial serial{"/dev/ttyTHS1"};

	bool flag_should_calibarte = false; // calibarte when this flag is true
	bool RUNNING = true;

	IMU()
	{
		serial.uartSet(460800, 8, 'N', 1); //set baudrate,...
	}

	void ParseData(char chr)
	{
		static char chrBuf[100];
		static unsigned char chrCnt = 0;
		chrBuf[chrCnt++] = chr;
		if (chrCnt < 11)
			return;
		signed short sData[4];
		unsigned char i;
		time_t now;
		char cTemp = 0;
		for (i = 0; i < 10; i++)
			cTemp += chrBuf[i]; // checksum
		if ((chrBuf[0] != 0x55) || ((chrBuf[1] & 0x50) != 0x50) ||
			(cTemp != chrBuf[10]))
		{
			printf("Error:%x %x\r\n", chrBuf[0], chrBuf[1]);
			memcpy(&chrBuf[0], &chrBuf[1], 10);
			chrCnt--;
			return;
		}

		memcpy(&sData[0], &chrBuf[2], 8);
		switch (chrBuf[1])
		{
		case 0x51:
			for (i = 0; i < 3; i++)
				_acc[i] = -(float)sData[i]  * (9.8* 16.0 / 32768.0);
			// time(&now);
			// printf("\r\nT:%s _acc:%6.3f %6.3f %6.3f ", asctime(localtime(&now)), _acc[0],_acc[1], _acc[2]);
			break;
		case 0x52:
			for (i = 0; i < 3; i++)
				_angular_vel[i] = (float)sData[i]  * (2000.0* M_PI/180. / 32768.0);
			//printf("angular_vel:%7.3f %7.3f %7.3f ", angular_vel[0], angular_vel[1], angular_vel[2]);
			break;
		case 0x53:
			for (i = 0; i < 3; i++)
				// euler_angle[i] = (float)sData[i] / 32768.0 * 180.0; // in deg
				euler_angle[i] = (float)sData[i] * (M_PI / 32768.0); // in rad
			//printf("A:%7.3f %7.3f %7.3f ", euler_angle[0], euler_angle[1], euler_angle[2]);
			break;
		case 0x54:
			for (i = 0; i < 3; i++)
				magnetic_field[i] = (float)sData[i];
			// printf("magnetic_field:%4.0f %4.0f %4.0f ", magnetic_field[0], magnetic_field[1], magnetic_field[2]);
			break;
		case 0x59:
			for (i=0;i<4;i++){ // q = qw+ qx*i+qy*j+qz*k == q[0]+ q[1]*i+q[2]*j+q[3]*k
				quat[i] = (float)sData[i]  * (1 / 32768.0);
			}
			// printf("quaternion:%4.2f %4.2f %4.2f %4.2f\n", quat[0], quat[1], quat[2], quat[2]);
			break;
		}
		chrCnt = 0;
	}

	void run()
	{
		// read matrix from file
		readMatrix(r_w1_w0,"./matrix.txt");

		char r_buf[1024];
		while (RUNNING)
		{
			ret = serial.recv(r_buf, 33); // 4 group of 11 bytes
			for (int i = 0; i < ret; i++)
			{
				ParseData(r_buf[i]);
			}
			// eulerToRotationMatrix(euler_angle[0], euler_angle[1], euler_angle[2], r_w0_imu);
			quaternionToRotationMatrix(quat[0],quat[1],quat[2],quat[3],r_w0_imu); // DO CONVERSION

			if(flag_should_calibarte){
				//do calibaration
				double r_w0_imu_inv[3][3];

				// printf("r_w0_imu");
				// printMatrix(r_w0_imu);

				findInverse(r_w0_imu_inv, r_w0_imu);

				// printf("r_w0_imu_inv");
				// printMatrix(r_w0_imu_inv);

				// printf("r_w1_imu_at_calibartion");
				// printMatrix(r_w1_imu_at_calibartion);
				
				matrixDot(r_w1_imu_at_calibartion,r_w0_imu_inv,r_w1_w0);

				// printf("r_w1_w0");
				// printMatrix(r_w1_w0);

				// printMatrix(r_w1_w0);
  				writeMatrix(r_w1_w0,"./matrix.txt");
				flag_should_calibarte = false;

				// matrixDot(r_w1_w0,r_w0_imu,r_w1_imu);
				// matrixDot(r_w1_imu,r_imu_rob,r_w1_rob);
				// printMatrix(r_w1_rob);
			}
			matrixDot(r_w1_w0,r_w0_imu,r_w1_imu);

			// compute robot transform relative to w1
			matrixDot(r_w1_imu,r_imu_rob,r_w1_rob); 


			findInverse(r_imu_w1, r_w1_imu);//compute r_imu_w1
			

			// Calculate acceleration, gravity subtracted
			matrixDotvector(r_imu_w1,_acc,acc);
			acc[2]=acc[2]+9.8;



			// Calculate angular velociety, robot 
			matrixDotvector(r_imu_rob,_angular_vel,angular_vel);

		}
		serial.close();
	}
};

class UdpReceiver
{
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
			try
			{
				int datalength = recvfrom(sock_recv, buf_UDP_recv, UDP_BUFFER_SIZE, 0, (struct sockaddr *)&hold_recv, &fromlen);
				msgpack::object_handle oh = msgpack::unpack(buf_UDP_recv, datalength);
				msgpack::object obj = oh.get();
				obj.convert(recv);
				msg_queue.push_front(recv);

				// printf("%d",recv.header);
				while (msg_queue.size() > 1)
				{
					msg_queue.pop_back();
				}
			}
			catch (const std::bad_cast &e)
			{ // msgpack obj.as<T> error
				printf("Error converting %s,line %d: %s \n", __FILE__, __LINE__, e.what());
			}
			catch (const msgpack::unpack_error e)
			{
				printf("Error converting %s,line %d: %s \n", __FILE__, __LINE__, e.what());
			}
			catch (const std::exception &e)
			{ // standard exceptions
				printf("Error converting %s,line %d: %s \n", __FILE__, __LINE__, e.what());
			}
		}
	}
};

int main()
{
	Teensycomm_struct_t msg_from_teensy; // A data struct received from Teensy
	Jetson_comm_struct_t msg_to_teensy;	 // A data struct sent to Teensy

	// TODO: THIS MAY NOT BE NEEDED
	// Initialize corresponding data structure
	for (int i = 0; i < MOTOR_NUM; ++i)
		desired_pos[i] = 0.0;

	UdpReceiver udp_receiver;
	std::thread recv_thread(&UdpReceiver::run, &udp_receiver); // udp receiving in a separate thread
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
	// Serial imu{"/dev/ttyUSB0"};
	// //        baudrate,...
	// imu.uartSet(460800, 8, 'N', 1);
	// char r_buf[1024];
	IMU imu{};

	std::thread imu_thread(&IMU::run, &imu); // udp receiving in a separate thread
	/********************* Serial for teensy****************************/
	Serial teensy_serial{"/dev/ttyACM0"};
	teensy_serial.uartSet(1000000, 8, 'N', 1);


	DataSend msg_to_pc[SEND_SIZE];
	DataSend data_send;
	std::deque<DataSend> send_buf;

	/******************************************************************/
	while (1)
	{
		if (udp_receiver.msg_queue.size() > 0)
		{
			MsgFromPC recv = udp_receiver.msg_queue.front();
			
			udp_receiver.msg_queue.pop_front();

			//quaternionToRotationMatrix(quat[0],quat[1],quat[2],quat[3],r_w0_imu);
			if (recv.header==10002){
				imu.flag_should_calibarte = true;
				printf("REQUESTED IMU RESET\n");
			}

			for (int i = 0; i < MOTOR_NUM; i++)
			{
				msg_to_teensy.comd[i] = recv.robot_command[i];
			}
			msg_to_teensy.header = recv.header;
			// send to teensy
			teensy_serial.sendStruct(msg_to_teensy);
		}
		
		//std::cout<<imu.euler[0]<<std::endl;
		// receive from teensy
		teensy_serial.recvStruct(msg_from_teensy);
		//std::cout<<"reached2"<<'\n';
		// printf("done sending to teensy 1212\n");

		// data_send.timestamps = comm->timestamps;

		unsigned long milliseconds_since_epoch =
			std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		data_send.timestamps = milliseconds_since_epoch;

		for (int j = 0; j < MOTOR_NUM; ++j)
		{
			// // Angle of motors:
			// data_send.joint_posraw[j] = comm.joint_pos[j];
			// Speed of motors:
			data_send.joint_vel[j] = msg_from_teensy.joint_vel[j];
			// Angle of motors:
			data_send.joint_pos[j * 2] = std::cos(msg_from_teensy.joint_pos[j]);
			data_send.joint_pos[j * 2 + 1] = std::sin(msg_from_teensy.joint_pos[j]);
			// Current of motors:
			data_send.joint_cur[j] = msg_from_teensy.joint_cur[j] * 9.375;
		}
		//std::cout<<comm->joint_cur[1]<<" "<<comm->joint_cur[1]<< " "<<comm->joint_cur[2]<<'\n';
		for (int i = 0; i < 3; ++i)
		{
			// msg_to_pc.mag[i] = comm->mag[i];
			data_send.acc[i] = imu.acc[i];
			data_send.angular_vel[i] = imu.angular_vel[i];
		}

		//std::cout<<data_send.joint_cur[2]<<'\n';

		data_send.orientation[0] = imu.r_w1_rob[0][0];
		data_send.orientation[1] = imu.r_w1_rob[1][0];
		data_send.orientation[2] = imu.r_w1_rob[2][0];
		data_send.orientation[3] = imu.r_w1_rob[0][1];
		data_send.orientation[4] = imu.r_w1_rob[1][1];
		data_send.orientation[5] = imu.r_w1_rob[2][1];

		// std::cout<<imu._angular_vel[0]<<","<<imu._angular_vel[1]<<","<<imu._angular_vel[2];//<<'\n';
		// std::cout<<imu.angular_vel[0]<<","<<imu.angular_vel[1]<<","<<imu.angular_vel[2]<<'\n';
		// std::cout<<imu.acc[0]<<","<<imu.acc[1]<<","<<imu.acc[2];//<<'\n';
		// std::cout<<imu.acc[0]<<","<<imu.acc[1]<<","<<imu.acc[2]<<'\n';

		// printMatrix(imu.r_w0_imu);
		// printMatrix(imu.r_w1_imu);
		// printMatrix(imu.r_w1_imu);


		// Foot sensor data process............
		convert_4voltages_to_mass(msg_from_teensy.foot_force);
		// on/off contact switch
		// data_send.foot_force[0]=leftfoot_mass;
		// data_send.foot_force[1]=rightfoot_mass;
		data_send.foot_force[0] = leftfoot_mass > 0.5 ? 1 : 0;
		data_send.foot_force[1] = rightfoot_mass > 0.5 ? 1 : 0;
		
		
		// for(int i=0;i<4;++i){
		// 	std::cout<<msg_from_teensy.foot_force[i]<<" ";
		// }
		// std::cout<<std::endl;
		// std::cout<<data_send.foot_force[0]<<" "<<data_send.foot_force[1]<<std::endl;
		// std::cout<<leftfoot_mass<<" "<<rightfoot_mass<<'\n';
		//.................

		send_buf.push_front(data_send);

		while (send_buf.size() > BUFFER_SIZE)
		{
			send_buf.pop_back();
		}

		if (send_buf.size() == BUFFER_SIZE)
		{
			for (int i = 0; i < SEND_SIZE; i++)
			{
				msg_to_pc[i] = send_buf[i * STEP_SIZE];
			}

			std::stringstream send_stream;
			msgpack::pack(send_stream, msg_to_pc);
			std::string const &data = send_stream.str();
			sendto(sock_send, data.c_str(), data.size(), 0, (struct sockaddr *)&client_send, sizeof(client_send));
		}
	}

	udp_receiver.RUNNING = false;
	if (recv_thread.joinable())
	{
		recv_thread.join();
	}

	imu.RUNNING = false;
	if (imu_thread.joinable())
	{
		imu_thread.join();
	}

	teensy_serial.close();

	return 0;
}
