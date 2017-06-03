// HumanSwarmSystem_Gazebo.cpp : 定義主控台應用程式的進入點。
//

#include "stdafx.h"
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define SERVER "192.168.19.2"
#define BUFLEN 1024  //Max length of buffer
#define PORT 1234   //The port on which to listen for incoming data
#define SAMPLING_TIME 0.01	//unit:second
#define DEVICE_NAME_1 "phantom1"
#define DEVICE_NAME_2 "phantom2"

//----------socket----------
#include <WinSock2.h>	//do not put behind <Windows.h>

//----------omni----------
#include <HD\hd.h>
#include <HDU\hduVector.h>
#include <HDU\hduError.h>
#include <time.h>
#include <Windows.h>// systemtime
#include <iostream>
#include <armadillo>// linear alegebra lib
#include <string.h>
#include <sstream>
#if defined(WIN32)
# include <conio.h>	// getch(), _kbhit()
#else
# include "conio.h"
# include <windows.h>
#endif
#include "../TimeDelayBuffer/TimeDelayBuffer.h"

using namespace std;
using namespace arma;
#pragma comment(lib,"ws2_32.lib") //Winsock Library

//--------global parameter--------
bool DELAY_SWITCH = true;
const int MasterRobotNum = 2;
const int MasterRobotDOF = 3;
const int SlaveRobotNum = 4;		//n
const int SlaveRobotDOF = 3;		//phi
const int TaskSpaceDimension = MasterRobotNum * MasterRobotDOF;	//m
const int TaskSpace_variance = TaskSpaceDimension / MasterRobotNum;
const int rows_ThetaHat_m = 8;
const double u_s_saturation = 7.0;
const double l_1_Omni = 133.0, l_2_Omni = 133.0;
const double MatElementTolerence = 0.00001;	//when using inv,pinv, the consequence less than this will equal to zero
mat X_m( TaskSpaceDimension, 1, fill::zeros );
mat X_m_last(TaskSpaceDimension, 1, fill::zeros);
mat X_m_temp(TaskSpaceDimension, 1, fill::zeros);
mat X_s( TaskSpaceDimension, 1, fill::zeros );
mat X_s_last(TaskSpaceDimension, 1, fill::zeros);
mat XDot_s( TaskSpaceDimension, 1, fill::zeros );
mat X_m_initial(TaskSpaceDimension, 1, fill::zeros);
mat X_s_initial(TaskSpaceDimension, 1, fill::zeros);
mat e_m_vec( TaskSpaceDimension, 1, fill::zeros );
mat e_m( MasterRobotDOF, MasterRobotNum, fill::zeros);
mat e_s( TaskSpaceDimension, 1, fill::zeros );
mat e_s_last(TaskSpaceDimension, 1, fill::zeros);
mat eDot_s( TaskSpaceDimension, 1, fill::zeros );
mat s_m( MasterRobotDOF, MasterRobotNum, fill::zeros );
mat v_m(size(s_m), fill::zeros);
mat v_m_last(size(s_m), fill::zeros);
mat a_m(size(s_m), fill::zeros);
mat s_s( SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros );
mat s_s_last(SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros);
mat q_m(size(s_m), fill::zeros);
mat q_m_last(size(s_m), fill::zeros);
mat qDot_m(size(s_m), fill::zeros);
mat q_s( SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros );
mat q_s_last(SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros);
mat qDot_s( SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros );
mat qDot_s_last(SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros);
mat qDoubleDot_s(SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros);
mat J_m1(MasterRobotDOF, TaskSpaceDimension/2, fill::zeros);
mat J_m2(size(J_m1), fill::zeros);
mat J_m1_last(size(J_m1), fill::zeros);
mat J_m2_last(size(J_m1), fill::zeros);
mat JDot_m1(size(J_m1), fill::zeros);
mat JDot_m2(size(J_m1), fill::zeros);
mat J_s( TaskSpaceDimension, SlaveRobotNum * SlaveRobotDOF, fill::zeros );
mat J_s_last(TaskSpaceDimension, SlaveRobotNum * SlaveRobotDOF, fill::zeros);
mat JDot_s(TaskSpaceDimension, SlaveRobotNum * SlaveRobotDOF, fill::zeros);
mat invJ_m, pinvJ_s;
mat Y_m1(MasterRobotDOF, rows_ThetaHat_m, fill::zeros);
mat Y_m2(size(Y_m1), fill::zeros);
mat ThetaHat_s( SlaveRobotNum * SlaveRobotDOF, 1 );
mat ThetaHatDot_s(SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros);
mat f_a( SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros );
mat u_m;
mat u_m_multiCol(size(s_m), fill::zeros);
mat forceOutput(size(s_m), fill::zeros);
mat u_s( SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros );
mat VelocityCommand_s(SlaveRobotNum * SlaveRobotDOF, 1, fill::zeros);
mat theta_m(MasterRobotDOF * MasterRobotNum, 1, fill::zeros);
mat ThetaHat_m(rows_ThetaHat_m, MasterRobotNum);
mat ThetaHatDot_m(rows_ThetaHat_m, MasterRobotNum, fill::zeros);

vec vk_p, vk_d, vk_t, valpha;
mat k_p, k_d, k_t, alpha;
double k_ss = 7.5 * 0.05;

HHD DeviceID_1, DeviceID_2;
HDdouble JointAngle_m1[MasterRobotDOF], JointAngle_m2[MasterRobotDOF];
/*
mat OmniBuf1(TaskSpace_variance, 3, fill::zeros);
mat OmniBuf2(TaskSpace_variance, 3, fill::zeros);
int OmniBufFlag1 = 0;
int OmniBufFlag2 = 0;

mat X_mFilterBuf(TaskSpaceDimension, 3, fill::zeros);
int X_mFilterBufFlag = 0;
mat X_sFilterBuf(TaskSpaceDimension, 3, fill::zeros);
int X_sFilterBufFlag = 0;
*/

//--------function--------
mat timeDerivative(mat last, mat now);
vec medianFilter(mat* dataBuf, mat* newData, int* BufFlag);
HDCallbackCode HDCALLBACK GetCmd1Callback(void *data);
HDCallbackCode HDCALLBACK GetCmd2Callback(void *data);
HDCallbackCode HDCALLBACK ForceOutput1Callback(void *data);
HDCallbackCode HDCALLBACK ForceOutput2Callback(void *data);
void printMatToTxt(FILE *dst, mat *src);
char* stringToChar_heap(string src);
void ForceController(int MasterNo, mat *J, mat *JDot, mat* Y);

//-----------------------------------------
int _tmain(int argc, _TCHAR* argv[])
{
	Sleep(500);	
	//--------initialize--------

	char* sendtoBuf = NULL;
	stringstream ss;
	string string_converted;

	ThetaHat_s.fill( 0.15 );
	valpha << 0.2 << 0.2 << 0.2 << 0.2 << 0.2 << 0.2;
	alpha = diagmat(valpha);
	/*
	vk_p << 0.0000125 << 0.0000125 << 0.0000125 << 0.0000075 << 0.0000075 << 0.0000075;
	k_p = diagmat(vk_p);
	vk_d << 0.00125 << 0.00125 << 0.00125 << 0.0001 << 0.0001 << 0.0001;
	k_d = diagmat(vk_d);
	vk_t << 0.02 << 0.02 << 0.02 << 0.2 << 0.2 << 0.2;
	k_t = diagmat(vk_t);*/
	vk_p << 0.000125 << 0.000125 << 0.000125 << 0.000075 << 0.000075 << 0.000075;
	k_p = diagmat(vk_p);
	//vk_d << 0.00125 << 0.00125 << 0.00125 << 0.0001 << 0.0001 << 0.0001;
	//k_d = diagmat(vk_d);
	vk_t << 0.2 << 0.2 << 0.2 << 0.8 << 0.8 << 0.8;
	//vk_t << 0.05 << 0.05 << 0.05 << 0.5 << 0.5 << 0.5;
	//vk_t << 0.02 << 0.02 << 0.02 << 0.2 << 0.2 << 0.2;
	k_t = diagmat(vk_t);

	//	---socket---
	struct sockaddr_in si_other;
	int s, slen = sizeof(si_other);
	WSADATA wsa;

	//	Initialise winsock
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	printf("Initialised.\n");

	//	create socket
	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
	{
		printf("socket() failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	//	setup address structure
	memset((char *)&si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);
	si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);
	/*	//for test
	cout << size(vk_t);
	q_s << 1 << endr 
	<< 2 << endr 
	<< 3 << endr 
	<< 4 << endr 
	<< 5 << endr
	<< 6 << endr 
	<< 7 << endr 
	<< 8 << endr 
	<< 9 << endr 
	<< 10 << endr
	<< 11 << endr 
	<< 12 << endr;
	qDot_s(0,0) = 1;
	qDot_s.print("qdots");
	test(&q_s, &qDot_s);
	q_s.print("qsafter");
	*/
	/*	//median filter test
	for (int i = 0; i < 4; i++)
	{
		vec v1(3,fill::randu), v2;
		//v1 << i + 0.1 << endr << i + 0.2 << endr << i + 0.3 << endr;
		v1.print("v1:");
		v2 = medianFilter(&OmniBuf1, &v1, &OmniBufFlag1);
		v2.print("v2:");
	}
	*/
	
	//	omni part

	/*
	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize first haptic device");
		fprintf(stderr, "Make sure the configuration\"%s\"exists\n", DEVICE_NAME_1);
		fprintf(stderr, "\nPress any key to quit.\n");
		getchar();
		exit(-1);
	}
	*/

	//	set omni & scheduler
	DeviceID_1 = hdInitDevice(DEVICE_NAME_1);
	hdMakeCurrentDevice(DeviceID_1);
	hdEnable(HD_FORCE_OUTPUT);
	DeviceID_2 = hdInitDevice(DEVICE_NAME_2);
	hdMakeCurrentDevice(DeviceID_2);
	hdEnable(HD_FORCE_OUTPUT);
	hdStartScheduler();
	HDSchedulerHandle Scheduler_ForceOutput1 = hdScheduleAsynchronous(ForceOutput1Callback, (void*) 0, HD_MAX_SCHEDULER_PRIORITY);
	HDSchedulerHandle Scheduler_ForceOutput2 = hdScheduleAsynchronous(ForceOutput2Callback, (void*) 0, HD_MAX_SCHEDULER_PRIORITY);

	//	use sendto() before using recvfrom() to implicitly bind
	for (int i = 0; i < SlaveRobotDOF * SlaveRobotNum; i++)
		ss << "0 ";
	sendtoBuf = stringToChar_heap(ss.str());
	ss.str("");
	ss.clear();
	
	if (sendto(s, sendtoBuf, BUFLEN, 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
	{
		printf("1sendto() failed with error code : %d", WSAGetLastError());
		system("PAUSE");
		exit(EXIT_FAILURE);
	}
	delete[] sendtoBuf;
	sendtoBuf = NULL;

	//	open txt file for recording
	FILE* X_m_no_delay_file = fopen("..\\record\\X_m_no_delay.txt", "w");
	FILE* X_m_file = fopen("..\\record\\X_m.txt", "w");
	FILE* X_s_file = fopen("..\\record\\X_s.txt", "w");
	FILE* q_s_file = fopen("..\\record\\q_s.txt", "w");
	FILE* qDot_s_file = fopen("..\\record\\qDot_s.txt", "w");
	FILE* u_s_file = fopen("..\\record\\u_s.txt", "w");
	FILE* e_s_file = fopen("..\\record\\e_s.txt", "w");
	FILE* delay_s_file = fopen("..\\record\\delay_s.txt", "w");
	
	//time delay buf
	DelayedBuffer *TimeDelayBuffer_m_to_s = NULL;
	DelayedData data_m_to_s;
	int delay_m_to_s;	//unit:ms

	//--------start--------

	int loopCount = 0;
	while (!_kbhit())
	{		
		/* 
		 * part1:get master and slave robots information to update q,qDot,X...
		 */

		char buf_from_slave[BUFLEN];
		int timer_loop_start, timer_loop_end;
		
		timer_loop_start = clock();
		delay_m_to_s = 0/*1000 + 1000 * sin(0.0001*timer_loop_start)*/;	// #delay
		ss << delay_m_to_s << endl;
		string_converted = ss.str();
		fprintf(delay_s_file, string_converted.c_str());
		ss.str("");
		ss.clear();		

		//	store value for calculating time derivative
		J_s_last = J_s;
		q_m_last = q_m;
		q_s_last = q_s;
		qDot_s_last = qDot_s;
		X_m_last = X_m;
		X_s_last = X_s;
		e_s_last = e_s;
		s_s_last = s_s;
		J_m1_last = J_m1;
		J_m2_last = J_m2;
		v_m_last = v_m;
		
		//	receive from master and then update X_m_temp
		hdScheduleSynchronous(GetCmd1Callback, (void*)0, HD_DEFAULT_SCHEDULER_PRIORITY);
		hdScheduleSynchronous(GetCmd2Callback, (void*)0, HD_DEFAULT_SCHEDULER_PRIORITY);
		
		//	receive from slave and then update X_s
		if (recvfrom(s, buf_from_slave, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) == SOCKET_ERROR)
		{
			printf("recvfrom() failed with error code : %d", WSAGetLastError());
			system("PAUSE");
			exit(EXIT_FAILURE);
		}
		//puts(buf_from_slave);
		
		ss << buf_from_slave;
		for (int num = 0; num < SlaveRobotNum; num++)
		{
			for (int axis = 0; axis < SlaveRobotDOF; axis++)
			{
				ss >> q_s(num * SlaveRobotDOF + axis, 0);
			}
		}
		ss.str("");
		ss.clear();

		//filter q_s
		if (abs(q_s - q_s_last).max() < 0.001)
			q_s = q_s_last;

		
		//	update X_s
		for (int axis = 0; axis < 3; axis++)
		{
			double PosSum = 0.0;
			for (int num = 0; num < SlaveRobotNum; num++)
			{
				PosSum += q_s(num * SlaveRobotDOF + axis, 0);
			}
			X_s(axis, 0) = PosSum / (double)SlaveRobotNum;

			double VarSum = 0.0;
			for (int num = 0; num < SlaveRobotNum; num++)
			{
				VarSum += pow(q_s(num * SlaveRobotDOF + axis, 0) - X_s(axis, 0), 2);
			}
			X_s(axis + TaskSpace_variance, 0) = VarSum / (double)SlaveRobotNum;
		}

		//	update X_m ,q_m
		X_m = X_m_temp;	//don't put this just after hdWaitForCompletion,
						//or the callback could not be completed yet
						//and the value might be changed unexpected
		q_m(0,0) = JointAngle_m1[0];	//master1
		q_m(1,0) = JointAngle_m1[1];
		q_m(2,0) = JointAngle_m1[2] - JointAngle_m1[1];
		q_m(0,1) = JointAngle_m2[0];	//master2
		q_m(1,1) = JointAngle_m2[1];
		q_m(2,1) = JointAngle_m2[2] - JointAngle_m2[1];

/*
		//filter q_s
		if (abs(X_s - X_s_last).max() < 0.01)
			X_s = X_s_last;
*/
		if (loopCount < 2)
		{
			X_m_initial = X_m;
			X_s_initial = X_s;
			//q_m_initail = q_m;
/*
			//median filter
			X_mFilterBuf.col(X_mFilterBufFlag) = X_m;
			X_mFilterBufFlag++;
			X_sFilterBuf.col(X_sFilterBufFlag) = X_s;
			X_sFilterBufFlag++;
		}
		else
		{
			X_m = medianFilter(&X_mFilterBuf, &X_m, &X_mFilterBufFlag);
			X_s = medianFilter(&X_sFilterBuf, &X_s, &X_sFilterBufFlag);
*/
		}

		X_m = X_m - X_m_initial + inv(alpha) * X_s_initial;		//	shift master coordinate to match slave initial condition
		

		for (int axis = 0; axis < SlaveRobotDOF; axis++)	//lower bound of command from omni
		{
			if (X_m(axis + TaskSpace_variance, 0) < 0.0)
			{
				X_m(axis + TaskSpace_variance, 0) = 0.0;
			}
		}
		printMatToTxt(X_m_no_delay_file, &X_m);
		if (loopCount == 0)
			TimeDelayBuffer_m_to_s = new DelayedBuffer(DelayedData(X_m, 0, 0));
		
		//time delay
		
		if (DELAY_SWITCH)
		{
			data_m_to_s = DelayedData(X_m, loopCount, timer_loop_start + delay_m_to_s);
			TimeDelayBuffer_m_to_s->addData(data_m_to_s);	//put current data into buffer
			X_m = TimeDelayBuffer_m_to_s->getData(timer_loop_start);	//get arrived data from buffer
			//cout << "size of buf: " << TimeDelayBuffer_m_to_s.buf.capacity() << endl;
		}
		
		printMatToTxt(X_m_file, &X_m);
		printMatToTxt(X_s_file, &X_s);
		printMatToTxt(q_s_file, &q_s);

		//	controller
		//	update time derivative of X,q,qDot

		if (loopCount != 0)	//skip at first because last value doesn't exist
		{
			XDot_s = timeDerivative(X_s_last, X_s);
			qDot_m = timeDerivative(q_m_last, q_m);
			qDot_s = timeDerivative(q_s_last, q_s);
			qDoubleDot_s = timeDerivative(qDot_s_last, qDot_s);
		}
		printMatToTxt(qDot_s_file, &qDot_s);
		
		/* 
		 * part2:update other parameters
		 */		
		e_m_vec = inv(alpha) *  X_s  - X_m;
		e_s = alpha * X_m  - X_s;
		e_m.col(0) = e_m_vec.rows(0, MasterRobotDOF - 1);
		e_m.col(1) = e_m_vec.rows(MasterRobotDOF, MasterRobotDOF * 2 - 1);

		eDot_s = timeDerivative(e_s_last, e_s);
		//e_s.t().print("e_s");
		//	J_m
		/*
		J_m(0, 0) = -l_1_Omni*cos(q_m(0,0)) * cos(q_m(1,0))	//master1
			- l_2_Omni * cos(q_m(0,0)) * cos(q_m(1,0)) * sin(q_m(2,0))
			- l_2_Omni * cos(q_m(0,0)) * sin(q_m(1,0)) * cos(q_m(2,0));
		J_m(0, 1) = l_2_Omni * sin(q_m(0,0)) * sin(q_m(1,0)) * sin(q_m(2,0))
			+ l_1_Omni *sin(q_m(0,0)) * sin(q_m(1,0))
			- l_2_Omni * sin(q_m(0,0)) *cos(q_m(1,0)) * cos(q_m(2,0));
		J_m(0,2) = -l_2_Omni * sin(q_m(0,0)) * cos(q_m(1,0)) * cos(q_m(2,0)) 
			+ l_2_Omni * sin(q_m(0,0)) * sin(q_m(1,0)) * sin(q_m(2,0));
		J_m(1,0) = 0;
		J_m(1, 1) = l_2_Omni * sin(q_m(1,0)) * cos(q_m(2,0)) 
			+ l_2_Omni * cos(q_m(1,0)) * sin(q_m(2,0)) 
			+ l_1_Omni  *  cos(q_m(1,0));
		J_m(1,2) = l_2_Omni * cos(q_m(1,0)) * sin(q_m(2,0)) 
			+ l_2_Omni * sin(q_m(1,0)) * cos(q_m(2,0));
		J_m(2,0) = -l_2_Omni * sin(q_m(0,0)) * sin(q_m(1,0)) * cos(q_m(2,0)) 
			- l_1_Omni * sin(q_m(0,0)) * cos(q_m(1,0)) 
			- l_2_Omni * sin(q_m(0,0)) * cos(q_m(1,0)) * sin(q_m(2,0));
		J_m(2,1) = l_2_Omni * cos(q_m(0,0)) * cos(q_m(1,0)) * cos(q_m(2,0)) 
			- l_2_Omni * cos(q_m(0,0)) * sin(q_m(1,0)) * sin(q_m(2,0)) 
			- l_1_Omni * cos(q_m(0,0)) * sin(q_m(1,0));
		J_m(2,2) = l_2_Omni * cos(q_m(0,0)) * cos(q_m(1,0)) * cos(q_m(2,0)) 
			- l_2_Omni * cos(q_m(0,0)) * sin(q_m(1,0)) * sin(q_m(2,0));

		J_m(3, 3) = -l_1_Omni*cos(q_m(0,1)) * cos(q_m(1,1))	//master2
			- l_2_Omni * cos(q_m(0,1)) * cos(q_m(1,1)) * sin(q_m(2,1))
			- l_2_Omni * cos(q_m(0,1)) * sin(q_m(1,1)) * cos(q_m(2,1));
		J_m(3, 4) = l_2_Omni * sin(q_m(0,1)) * sin(q_m(1,1)) * sin(q_m(2,1))
			+ l_1_Omni *sin(q_m(0,1)) * sin(q_m(1,1))
			- l_2_Omni * sin(q_m(0,1)) *cos(q_m(1,1)) * cos(q_m(2,1));
		J_m(3, 5) = -l_2_Omni * sin(q_m(0,1)) * cos(q_m(1,1)) * cos(q_m(2,1))
			+ l_2_Omni * sin(q_m(0,1)) * sin(q_m(1,1)) * sin(q_m(2,1));
		J_m(4, 3) = 0;
		J_m(4, 4) = l_2_Omni * sin(q_m(1,1)) * cos(q_m(2,1))
			+ l_2_Omni * cos(q_m(1,1)) * sin(q_m(2,1))
			+ l_1_Omni  *  cos(q_m(1,1));
		J_m(4, 5) = l_2_Omni * cos(q_m(1,1)) * sin(q_m(2,1))
			+ l_2_Omni * sin(q_m(1,1)) * cos(q_m(2,1));
		J_m(5, 3) = -l_2_Omni * sin(q_m(0,1)) * sin(q_m(1,1)) * cos(q_m(2,1))
			- l_1_Omni * sin(q_m(0,1)) * cos(q_m(1,1))
			- l_2_Omni * sin(q_m(0,1)) * cos(q_m(1,1)) * sin(q_m(2,1));
		J_m(5, 4) = l_2_Omni * cos(q_m(0,1)) * cos(q_m(1,1)) * cos(q_m(2,1))
			- l_2_Omni * cos(q_m(0,1)) * sin(q_m(1,1)) * sin(q_m(2,1))
			- l_1_Omni * cos(q_m(0,1)) * sin(q_m(1,1));
		J_m(5, 5) = l_2_Omni * cos(q_m(0,1)) * cos(q_m(1,1)) * cos(q_m(2,1))
			- l_2_Omni * cos(q_m(0,1)) * sin(q_m(1,1)) * sin(q_m(2,1));
		*/
		printMatToTxt(e_s_file, &e_s);

		mat SlaveAveragePosition(SlaveRobotDOF, 1, fill::zeros);
		for (int axis = 0; axis < SlaveRobotDOF; axis++)
		{
			for (int num = 0; num < SlaveRobotNum; num++)
			{
				SlaveAveragePosition(axis,0) += q_s(axis + num * SlaveRobotDOF,0);
			}
			SlaveAveragePosition(axis,0) /= (double)SlaveRobotNum;
		}
		
		//	primary task
		for (int row = 0; row < TaskSpaceDimension; row++)
		{
			//	average position
			//	1/n	0	0	...
			//	0	1/n	0	...
			//	0	0	1/n	...
			if (row < 3)
			{
				for (int col = 0; col < SlaveRobotNum; col++)
				{
					J_s(row, col*SlaveRobotDOF + row % SlaveRobotDOF) = 1.0 / (double)SlaveRobotNum;
				}
			}
			//	variance
			//	v_x	0	0	...
			//	0	v_y	0	...
			//	0	0	v_z	...
			//	v_i = 2/(phi*n)*(q_si-avg_i), i=x,y,z
			else
			{
				for (int col = 0; col < SlaveRobotNum; col++)
				{
					J_s(row, col*SlaveRobotDOF + row % SlaveRobotDOF) 
						= 2 / (double)(/*SlaveRobotDOF**/SlaveRobotNum)*(q_s(col*SlaveRobotDOF + row % SlaveRobotDOF, 0) - SlaveAveragePosition(row % SlaveRobotDOF, 0));
				}
			}
		}

		//invJ_m = inv( J_m, MatElementTolerence );
		pinvJ_s = pinv( J_s, MatElementTolerence );
		//	secondary task
		//	maximize inner distance
		mat InnerDistancef_a(SlaveRobotDOF*SlaveRobotNum, 1, fill::zeros);	//partial f_d(2.19)
		mat Distance(SlaveRobotNum, SlaveRobotNum, fill::zeros);
		for (int row = 0; row < SlaveRobotNum; row++)
		{
			for (int col = 0; col < SlaveRobotNum; col++)
			{
				for (int axis = 0; axis < SlaveRobotDOF; axis++)
				{
					if (row == col)break;
					Distance(row, col) += pow(q_s(col*SlaveRobotDOF + axis, 0) - q_s(row*SlaveRobotDOF + axis, 0), 2);
				}
				Distance(row, col) = sqrt(Distance(row, col));
			}
		}
		for (int robot = 0; robot < SlaveRobotNum; robot++)
		{
			for (int axis = 0; axis < SlaveRobotDOF; axis++)
			{
				for (int other = 0; other < SlaveRobotNum; other++)
				{
					if (robot != other)
					{
						InnerDistancef_a(robot*SlaveRobotDOF + axis, 0) 
							+= (q_s(robot*SlaveRobotDOF + axis, 0) - q_s(other*SlaveRobotDOF + axis, 0)) / Distance(robot, other);
					}
				}
			}
		}
		InnerDistancef_a *= -2.0 / (double)(SlaveRobotNum*(SlaveRobotNum - 1));
		//	collision avoidance
		mat CollisionAvoidancef_a(SlaveRobotDOF*SlaveRobotNum, 1, fill::zeros);

		f_a = -InnerDistancef_a * 10.0 -CollisionAvoidancef_a * 0.0;
		
		JDot_s = timeDerivative(J_s_last, J_s);
		//s_m = -inv(J_m) * k_t * e_m + qDot_m;
		s_s = -pinvJ_s * k_t * e_s + qDot_s - ( eye( SlaveRobotNum * SlaveRobotDOF, SlaveRobotNum * SlaveRobotDOF ) - pinvJ_s * J_s ) * f_a;
		//Y_s = pinv(JDot_s, MatElementTolerence) * k_t * e_s + pinvJ_s * k_t * eDot_s + qDoubleDot_s;
		//Y_s = timeDerivative(s_s_last, s_s);
		//ThetaHatDot_s = -1 * diagmat(Y_s).t() * s_s;
		if (loopCount >= 5)
		{
			//thetaHat_m += thetaHatDot_m * SAMPLING_TIME;
			//u_m = ;
			//u_s = diagmat(Y_s) * ThetaHat_s - k_ss * s_s - trans(J_s) * k_p * (-k_t * e_s + XDot_s);
			u_s = (qDoubleDot_s - timeDerivative(s_s_last, s_s)) - k_ss * s_s - trans(J_s) * k_p * J_s * s_s;

			//	u saturation
			for (int i = 0; i < u_s.n_rows; i++)
			{
				if (abs(u_s(i, 0)) > u_s_saturation)
				{
					if (u_s(i, 0) > 0)
						u_s(i, 0) = u_s_saturation;
					else
						u_s(i, 0) = -u_s_saturation;

				}
			}

			VelocityCommand_s = VelocityCommand_s + u_s * SAMPLING_TIME;	//integrate
		}
		
		for (int i = 0; i < VelocityCommand_s.n_rows; i++)
		{
			if (abs(VelocityCommand_s(i, 0)) < 0.00001)
				VelocityCommand_s(i, 0) = 0.0;
		}

		printMatToTxt(u_s_file, &u_s);
		ForceController(0, &J_m1, &JDot_m1, &Y_m1);
		ForceController(1, &J_m2, &JDot_m2, &Y_m2);

		JDot_m1 = timeDerivative(J_m1_last, J_m1);
		JDot_m2 = timeDerivative(J_m2_last, J_m2);
		/* 
		 * part3:send command to robots
		 */
		loopCount++;
		// to master
		/*
		if (!hdWaitForCompletion(Scheduler_ForceOutput1, HD_WAIT_CHECK_STATUS) || !hdWaitForCompletion(Scheduler_ForceOutput2, HD_WAIT_CHECK_STATUS))
		{
			fprintf(stderr, "Press any key to quit.\n");
			_getch();
			break;
		}
		*/
		//ForceOutput1Callback((void*)0);

		//	get enough data for median filter in the beginning and send command to keep remote UDP working(for blocking)
		
		/*
		if (loopCount < 5)
		{
			if (sendto(s, sFirstSend.c_str(), BUFLEN, 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
			{
				printf("sendto() failed with error code : %d", WSAGetLastError());
				system("PAUSE");
				exit(EXIT_FAILURE);
			}
			continue;
		}*/
		//	to slave
		
		for (int num = 0; num < SlaveRobotNum; num++)
		{
			for (int axis = 0; axis < SlaveRobotDOF; axis++)
			{
				ss << VelocityCommand_s(num * SlaveRobotDOF + axis, 0) << " ";
			}
		}
		sendtoBuf = stringToChar_heap(ss.str());
		//	clear stringstream
		ss.str("");
		ss.clear();
		if (sendto(s, sendtoBuf, BUFLEN, 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
		{
			printf("2sendto() failed with error code : %d\n", WSAGetLastError());
			break;
			//system("PAUSE");
			//exit(EXIT_FAILURE);
		}
		delete[] sendtoBuf;
		sendtoBuf = NULL;
		//cout << loopCount << endl;
		//	make fix sampling time
		timer_loop_end = clock();
		int until_sampling = SAMPLING_TIME * 1000 - (timer_loop_end - timer_loop_start);
		if (until_sampling > 0)	//make sure the case loop time is not greater than sampling time
			Sleep(until_sampling);
	}

	//--------end--------
	delete TimeDelayBuffer_m_to_s;
	TimeDelayBuffer_m_to_s = NULL;
	hdStopScheduler();
	hdUnschedule(Scheduler_ForceOutput1);
	hdUnschedule(Scheduler_ForceOutput2);
	hdDisableDevice(DeviceID_1);
	hdDisableDevice(DeviceID_2);
	closesocket(s);
	WSACleanup();
	_fcloseall();

	system("PAUSE");
	return 0;
}

mat timeDerivative(mat last, mat now)
{
	//double SamplingTime = 0.1;
	return (now - last) / SAMPLING_TIME;
}

HDCallbackCode HDCALLBACK GetCmd1Callback(void *data)
{
	HDdouble command[TaskSpace_variance];

	//	read command from omni
	hdBeginFrame(DeviceID_1);
	hdGetDoublev(HD_CURRENT_POSITION, command);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, JointAngle_m1);
	for (int axis = 0; axis < TaskSpace_variance; axis++)
	{
		X_m_temp(axis, 0) = command[axis];
	}
	
	hdEndFrame(DeviceID_1);
	return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK GetCmd2Callback(void *data)
{
	HDdouble command[TaskSpace_variance];

	//	read command from omni
	hdBeginFrame(DeviceID_2);
	hdGetDoublev(HD_CURRENT_POSITION, command);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, JointAngle_m2);
	for (int axis = 0; axis < TaskSpace_variance; axis++)
	{
		X_m_temp(3 + axis, 0) = command[axis];
	}
	
	hdEndFrame(DeviceID_2);
	return HD_CALLBACK_DONE;
}

vec medianFilter(mat* dataBuf, mat* newData, int* BufFlag)
{
	/*
	 * store 3 groups of values and output the median
	 */
	rowvec Sorting((*dataBuf).n_cols);
	vec result((*newData).n_rows);

	(*dataBuf).col(*BufFlag) = *newData;
	for (int  i = 0; i < (*dataBuf).n_rows; i++)
	{
		Sorting = sort((*dataBuf).row(i));
		result(i) = Sorting(Sorting.n_cols / 2);
	}
	(*BufFlag) = ((*BufFlag) + 1) % (*dataBuf).n_cols;
	return result;
}

void printMatToTxt(FILE *dst, mat *src)
{
	for (int i = 0; i < src->n_rows; i++)
	{
		fprintf(dst, "%f", src->at(i,0));
		if (i != src->n_rows - 1)
			fprintf(dst, " ");
	}
	fprintf(dst, "\n");
}

char* stringToChar_heap(string src)
{
	//	use heap memory for sendto() buf
	//	remember to delete
	char* buf = new char[src.size()+1];
	strcpy(buf, src.c_str());
	return buf;
}

HDCallbackCode HDCALLBACK ForceOutput1Callback(void *data)
{
	HDdouble force[MasterRobotDOF];
	
	hdBeginFrame(DeviceID_1);
	for (int axis = 0; axis < MasterRobotDOF; axis++)
		force[axis] = /*sin(0.0001*clock())*/forceOutput(axis, 0);
	hdMakeCurrentDevice(DeviceID_1);
	hdSetDoublev(HD_CURRENT_FORCE, force);
	hdEndFrame(DeviceID_1);

	return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK ForceOutput2Callback(void *data)
{
	HDdouble force[MasterRobotDOF];

	hdBeginFrame(DeviceID_2);
	for (int axis = 0; axis < MasterRobotDOF; axis++)
		force[axis] = /*sin(0.0001*clock())*/forceOutput(axis, 0);
	hdMakeCurrentDevice(DeviceID_2);
	hdSetDoublev(HD_CURRENT_FORCE, force);
	hdEndFrame(DeviceID_2);

	return HD_CALLBACK_CONTINUE;
}

void ForceController(int MasterNo, mat* J, mat* JDot, mat* Y)
{
	vec k_1_vec, k_2_vec;
	mat gravityCompensate(size(s_m.col(MasterNo)), fill::zeros);
	mat invJ, k_1, k_2;
	mat ini_cancel = -(-X_m_initial + inv(alpha) * X_s_initial);;

	k_1_vec << 5 * 2.3 << 6 * 2.0 << 7 * 2.0;
	k_2_vec << 0.00017 << 0.00025 << 0.00025;
	//k_1_vec << 1 << 1 << 1;
	//k_2_vec << 1 << 1 << 1;
	k_1 = diagmat(k_1_vec);
	k_2 = diagmat(k_2_vec);

	(*J)(0, 0) = -l_1_Omni*cos(q_m(0,MasterNo)) * cos(q_m(1,MasterNo))
		- l_2_Omni * cos(q_m(0,MasterNo)) * cos(q_m(1,MasterNo)) * sin(q_m(2,MasterNo))
		- l_2_Omni * cos(q_m(0,MasterNo)) * sin(q_m(1,MasterNo)) * cos(q_m(2,MasterNo));
	(*J)(0, 1) = l_2_Omni * sin(q_m(0,MasterNo)) * sin(q_m(1,MasterNo)) * sin(q_m(2,MasterNo))
		+ l_1_Omni *sin(q_m(0,MasterNo)) * sin(q_m(1,MasterNo))
		- l_2_Omni * sin(q_m(0,MasterNo)) *cos(q_m(1,MasterNo)) * cos(q_m(2,MasterNo));
	(*J)(0, 2) = -l_2_Omni * sin(q_m(0,MasterNo)) * cos(q_m(1,MasterNo)) * cos(q_m(2,MasterNo))
		+ l_2_Omni * sin(q_m(0,MasterNo)) * sin(q_m(1,MasterNo)) * sin(q_m(2,MasterNo));
	(*J)(1, 0) = 0;
	(*J)(1, 1) = l_2_Omni * sin(q_m(1,MasterNo)) * cos(q_m(2,MasterNo))
		+ l_2_Omni * cos(q_m(1,MasterNo)) * sin(q_m(2,MasterNo))
		+ l_1_Omni  *  cos(q_m(1,MasterNo));
	(*J)(1, 2) = l_2_Omni * cos(q_m(1,MasterNo)) * sin(q_m(2,MasterNo))
		+ l_2_Omni * sin(q_m(1,MasterNo)) * cos(q_m(2,MasterNo));
	(*J)(2, 0) = -l_2_Omni * sin(q_m(0,MasterNo)) * sin(q_m(1,MasterNo)) * cos(q_m(2,MasterNo))
		- l_1_Omni * sin(q_m(0,MasterNo)) * cos(q_m(1,MasterNo))
		- l_2_Omni * sin(q_m(0,MasterNo)) * cos(q_m(1,MasterNo)) * sin(q_m(2,MasterNo));
	(*J)(2, 1) = l_2_Omni * cos(q_m(0,MasterNo)) * cos(q_m(1,MasterNo)) * cos(q_m(2,MasterNo))
		- l_2_Omni * cos(q_m(0,MasterNo)) * sin(q_m(1,MasterNo)) * sin(q_m(2,MasterNo))
		- l_1_Omni * cos(q_m(0,MasterNo)) * sin(q_m(1,MasterNo));
	(*J)(2, 2) = l_2_Omni * cos(q_m(0,MasterNo)) * cos(q_m(1,MasterNo)) * cos(q_m(2,MasterNo))
		- l_2_Omni * cos(q_m(0,MasterNo)) * sin(q_m(1,MasterNo)) * sin(q_m(2,MasterNo));

	invJ = inv(*J);
	v_m.col(MasterNo) = invJ * (e_m.col(MasterNo) + ini_cancel.rows(MasterNo * MasterRobotDOF, MasterNo * MasterRobotDOF + MasterRobotDOF - 1));
	s_m.col(MasterNo) = qDot_m.col(MasterNo) - v_m.col(MasterNo);
	a_m.col(MasterNo) = timeDerivative(v_m_last.col(MasterNo), v_m.col(MasterNo));

	(*Y)(0, 0) = a_m(0, MasterNo) * cos(2 * q_m(1, MasterNo)) 
		- sin(2 * q_m(1, MasterNo))*(v_m(1, MasterNo) * qDot_m(0, MasterNo) - v_m(0, MasterNo) * qDot_m(1, MasterNo));
	(*Y)(0, 1) = -sin(2 * q_m(1, MasterNo) + 2 * q_m(2, MasterNo))*qDot_m(0, MasterNo) * (v_m(1, MasterNo) + v_m(2, MasterNo)) 
		- v_m(0, MasterNo) * sin(2 * q_m(1, MasterNo) + 2 * q_m(2, MasterNo))*(qDot_m(1, MasterNo) - qDot_m(2, MasterNo)) 
		+ a_m(0, MasterNo) * cos(2 * q_m(1, MasterNo) + 2 * q_m(2, MasterNo));
	(*Y)(0, 2) = a_m(0, MasterNo) * (sin(2 * q_m(1, MasterNo) + q_m(2, MasterNo)) + sin(q_m(2, MasterNo))) 
		+ 0.5*cos(q_m(2, MasterNo))*(v_m(0, MasterNo) * qDot_m(2, MasterNo) + v_m(2, MasterNo) * qDot_m(0, MasterNo)) 
		+ cos(2 * q_m(1, MasterNo) + q_m(2, MasterNo))*(v_m(1, MasterNo) * qDot_m(0, MasterNo) + 0.5*v_m(2, MasterNo) * qDot_m(0, MasterNo) + v_m(0, MasterNo) * qDot_m(1, MasterNo) + 0.5*v_m(0, MasterNo) * qDot_m(2, MasterNo));
	(*Y)(0, 3) = a_m(0, MasterNo);
	(*Y)(0, 4) = 0;
	(*Y)(0, 5) = 0;
	(*Y)(0, 6) = 0;
	(*Y)(0, 7) = 0;

	(*Y)(1, 0) = v_m(0, MasterNo) * sin(2 * q_m(1, MasterNo))*qDot_m(0, MasterNo);
	(*Y)(1, 1) = v_m(0, MasterNo) * sin(2 * q_m(1, MasterNo) + 2 * q_m(2, MasterNo))*qDot_m(0, MasterNo);
	(*Y)(1, 2) = sin(q_m(2, MasterNo))*(2 * a_m(1, MasterNo) + a_m(2, MasterNo)) 
		- v_m(0, MasterNo) * cos(2 * q_m(1, MasterNo) + q_m(2, MasterNo))*qDot_m(0, MasterNo) 
		+ cos(q_m(2, MasterNo))*(v_m(2, MasterNo) * qDot_m(1, MasterNo) + v_m(2, MasterNo) * qDot_m(2, MasterNo) + v_m(1, MasterNo) * qDot_m(2, MasterNo));
	(*Y)(1, 3) = 0;
	(*Y)(1, 4) = a_m(1, MasterNo);
	(*Y)(1, 5) = a_m(2, MasterNo);
	(*Y)(1, 6) = sin(q_m(1, MasterNo) + q_m(2, MasterNo));
	(*Y)(1, 7) = cos(q_m(1, MasterNo));


	(*Y)(2, 0) = 0;
	(*Y)(2, 1) = v_m(0, MasterNo) * sin(2 * q_m(1, MasterNo) + 2 * q_m(2, MasterNo))*qDot_m(0, MasterNo);
	(*Y)(2, 2) = a_m(1, MasterNo) * sin(q_m(2, MasterNo)) 
		- (0.5*v_m(0, MasterNo) * cos(2 * q_m(1, MasterNo) + q_m(2, MasterNo)) + 0.5*v_m(0, MasterNo) * cos(q_m(2, MasterNo)))*qDot_m(0, MasterNo) 
		- cos(q_m(2, MasterNo))*v_m(1, MasterNo) * qDot_m(1, MasterNo);
	(*Y)(2, 3) = 0;
	(*Y)(2, 4) = 0;
	(*Y)(2, 5) = a_m(1, MasterNo) + a_m(2, MasterNo);
	(*Y)(2, 6) = sin(q_m(1, MasterNo) + q_m(2, MasterNo));
	(*Y)(2, 7) = 0;

	ThetaHatDot_m.col(MasterNo) = -Y->t() * s_m.col(MasterNo);
	ThetaHat_m.col(MasterNo) = ThetaHat_m.col(MasterNo) + ThetaHatDot_m.col(MasterNo) * SAMPLING_TIME;	//integrate
		
	gravityCompensate(1, 0) = 100 * sin(q_m(1, MasterNo) + q_m(2, MasterNo)) + 80 * cos(q_m(1, MasterNo));
	gravityCompensate(2, 0) = 100 * sin(q_m(1, MasterNo) + q_m(2, MasterNo));

	u_m_multiCol.col(MasterNo) = (*Y)*ThetaHat_m.col(MasterNo) - k_1 * s_m.col(MasterNo) - k_2 * J->t() * (*J) * s_m.col(MasterNo);
	forceOutput.col(MasterNo) = invJ.t() * (u_m_multiCol.col(MasterNo) + gravityCompensate);
	forceOutput.col(MasterNo).t().print("f:");
}