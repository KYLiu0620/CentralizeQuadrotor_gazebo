// HumanSwarmSystem_Gazebo.cpp : 定義主控台應用程式的進入點。
//


#include "stdafx.h"
//----------socket----------
#include <WinSock2.h>	//do not put behind <Windows.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#define SERVER "192.168.19.47"
#define BUFLEN 1024  //Max length of buffer
#define PORT 1234   //The port on which to listen for incoming data
#define SAMPLING_TIME 0.1	//unit:second

//----------omni----------
#include <HD\hd.h>
#include <HDU\hduVector.h>
#include <HDU\hduError.h>
#define DEVICE_NAME_1 "phantom1"
#define DEVICE_NAME_2 "phantom2"

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

using namespace std;
using namespace arma;

//--------global parameter--------
const int RobotNum = 4;				//n
const int TaskSpaceDimension = 6;	//m
const int RobotDOF = 3;				//phi
const double alpha = 0.2;			//work envolope scaling factor
const double MatElementTolerence = 0.00001;	//when using inv,pinv, the consequence less than this will equal to zero
mat X_m( TaskSpaceDimension, 1, fill::zeros );
mat X_m_last(TaskSpaceDimension, 1, fill::zeros);
mat X_m_temp(TaskSpaceDimension, 1, fill::zeros);
mat X_s( TaskSpaceDimension, 1, fill::zeros );
mat X_s_last(TaskSpaceDimension, 1, fill::zeros);
mat XDot_s( TaskSpaceDimension, 1, fill::zeros );
mat X_m_initial(TaskSpaceDimension, 1, fill::zeros);
mat X_s_initial(TaskSpaceDimension, 1, fill::zeros);
mat e_m( TaskSpaceDimension, 1, fill::zeros );
mat e_s( TaskSpaceDimension, 1, fill::zeros );
mat e_s_last(TaskSpaceDimension, 1, fill::zeros);
mat eDot_s( TaskSpaceDimension, 1, fill::zeros );
mat s_m( RobotNum * RobotDOF, 1, fill::zeros );
mat s_s( RobotNum * RobotDOF, 1, fill::zeros );
mat s_s_last(RobotNum * RobotDOF, 1, fill::zeros);
mat q_m(TaskSpaceDimension, 1, fill::zeros);
mat q_m_last(TaskSpaceDimension, 1, fill::zeros);
mat qDot_m(TaskSpaceDimension, 1, fill::zeros);
mat q_s( RobotNum * RobotDOF, 1, fill::zeros );
mat q_s_last(RobotNum * RobotDOF, 1, fill::zeros);
mat qDot_s( RobotNum * RobotDOF, 1, fill::zeros );
mat qDot_s_last(RobotNum * RobotDOF, 1, fill::zeros);
mat qDoubleDot_s(RobotNum * RobotDOF, 1, fill::zeros);
mat J_m( TaskSpaceDimension, TaskSpaceDimension, fill::eye );
mat J_s( TaskSpaceDimension, RobotNum * RobotDOF, fill::zeros );
mat J_s_last(TaskSpaceDimension, RobotNum * RobotDOF, fill::zeros);
mat JDot_s(TaskSpaceDimension, RobotNum * RobotDOF, fill::zeros);
mat invJ_m, pinvJ_s;
mat Y_s;
mat thetaHat_s( RobotNum * RobotDOF, 1 );
mat thetaHatDot_s(RobotNum * RobotDOF, 1, fill::zeros);
mat f_a( RobotNum * RobotDOF, 1, fill::zeros );
mat u_m;
mat u_s( RobotNum * RobotDOF, 1, fill::zeros );

mat VelocityCommand_s(RobotNum * RobotDOF, 1, fill::zeros);

vec vk_p, vk_d, vk_t;
mat k_p, k_d, k_t;
double k_ss = 7.5 * 0.05;

HHD DeviceID_1, DeviceID_2;/*
mat OmniBuf1(TaskSpaceDimension / 2, 3, fill::zeros);
mat OmniBuf2(TaskSpaceDimension / 2, 3, fill::zeros);
int OmniBufFlag1 = 0;
int OmniBufFlag2 = 0;*/
mat X_mFilterBuf(TaskSpaceDimension, 3, fill::zeros);
int X_mFilterBufFlag = 0;
mat X_sFilterBuf(TaskSpaceDimension, 3, fill::zeros);
int X_sFilterBufFlag = 0;

double l_1_Omni = 133.0, l_2_Omni = 133.0;
//--------function--------
mat timeDerivative(mat last, mat now);
vec medianFilter(mat* dataBuf, mat* newData, int* BufFlag);
HDCallbackCode HDCALLBACK GetCmd1Callback(void *data);
HDCallbackCode HDCALLBACK GetCmd2Callback(void *data);
void printToTxt(FILE *dst, mat *src);
//-----------------------------------------
int _tmain(int argc, _TCHAR* argv[])
{
	Sleep(500);

	//--------initialize--------
	thetaHat_s.fill( 0.15 );
	
	vk_p << 0.0000125 << 0.0000125 << 0.0000125 << 0.0000075 << 0.0000075 << 0.0000075;
	k_p = diagmat(vk_p);
	vk_d << 0.00125 << 0.00125 << 0.00125 << 0.0001 << 0.0001 << 0.0001;
	k_d = diagmat(vk_d);
	vk_t << 0.02 << 0.02 << 0.02 << 0.2 << 0.2 << 0.2;
	k_t = diagmat(vk_t);
	
	
	/*
	vk_p << 0.000125 << 0.000125 << 0.000125 << 0.000075 << 0.000075 << 0.000075;
	k_p = diagmat(vk_p);
	vk_d << 0.00125 << 0.00125 << 0.00125 << 0.0001 << 0.0001 << 0.0001;
	k_d = diagmat(vk_d);
	vk_t << 0.1 << 0.05 << 0.1 << 1 << 0.5 << 1;
	//vk_t << 0.05 << 0.05 << 0.05 << 0.5 << 0.5 << 0.5;
	//vk_t << 0.02 << 0.02 << 0.02 << 0.2 << 0.2 << 0.2;
	k_t = diagmat(vk_t); */
	
	/*
	vk_p << 0.0001 << 0.0001 << 0.0001 << 0.00005 << 0.00005 << 0.00005;
	k_p = diagmat(vk_p);
	vk_d << 0.00125 << 0.00125 << 0.00125 << 0.0001 << 0.0001 << 0.0001;
	k_d = diagmat(vk_d);
	vk_t << 0.02 << 0.02 << 0.02 << 0.1 << 0.1 << 0.1;
	k_t = diagmat(vk_t);
	*/

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
	DeviceID_2 = hdInitDevice(DEVICE_NAME_2);
	hdStartScheduler();
	HDSchedulerHandle Scheduler_GetCmd1 = hdScheduleAsynchronous(GetCmd1Callback, (void*) 0, HD_MAX_SCHEDULER_PRIORITY);
	HDSchedulerHandle Scheduler_GetCmd2 = hdScheduleAsynchronous(GetCmd2Callback, (void*) 0, HD_MAX_SCHEDULER_PRIORITY);
	
	//	use sendto() before using recvfrom() to implicitly bind
	stringstream ssFirstSend;
	for (int i = 0; i < RobotDOF * RobotNum; i++)
		ssFirstSend << "0 ";
	string sFirstSend = ssFirstSend.str();
	if (sendto(s, sFirstSend.c_str(), BUFLEN, 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
	{
		printf("sendto() failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	//	open txt file for recording
	FILE* X_m_file = fopen("..\\record\\X_m.txt", "w");
	FILE* X_s_file = fopen("..\\record\\X_s.txt", "w");
	FILE* q_s_file = fopen("..\\record\\q_s.txt", "w");
	FILE* qDot_s_file = fopen("..\\record\\qDot_s.txt", "w");
	FILE* u_s_file = fopen("..\\record\\u_s.txt", "w");
	FILE* e_s_file = fopen("..\\record\\e_s.txt", "w");
	//--------start--------
	int loopCount = 0;
	while (!_kbhit())
	{
		
		/* 
		 * part1:get master and slave robots information to update q,qDot,X...
		 */

		SYSTEMTIME LoopStart;
		GetLocalTime(&LoopStart);
		stringstream ss_from_slave;
		char buf_from_slave[BUFLEN];

		//	store value for calculating time derivative
		J_s_last = J_s;
		q_m_last = q_m;
		q_s_last = q_s;
		qDot_s_last = qDot_s;
		X_m_last = X_m;
		X_s_last = X_s;
		e_s_last = e_s;
		s_s_last = s_s;

		//	receive from master and then update X_m_temp
		if (!hdWaitForCompletion(Scheduler_GetCmd1, HD_WAIT_CHECK_STATUS) || !hdWaitForCompletion(Scheduler_GetCmd2, HD_WAIT_CHECK_STATUS))
		{
			fprintf(stderr, "Press any key to quit.\n");
			_getch();
			break;
		}
		
		//	receive from slave and then update X_s
		if (recvfrom(s, buf_from_slave, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) == SOCKET_ERROR)
		{
			printf("recvfrom() failed with error code : %d", WSAGetLastError());
			system("PAUSE");
			exit(EXIT_FAILURE);
		}
		//puts(buf_from_slave);

		ss_from_slave << buf_from_slave;
		for (int num = 0; num < RobotNum; num++)
		{
			for (int axis = 0; axis < RobotDOF; axis++)
			{
				ss_from_slave >> q_s(num * RobotDOF + axis, 0);
			}
		}
		
		ss_from_slave.str("");
		ss_from_slave.clear();

		//filter q_s
		if (abs(q_s - q_s_last).max() < 0.001)
			q_s = q_s_last;


		//	update X_s
		for (int axis = 0; axis < 3; axis++)
		{
			double PosSum = 0.0;
			for (int num = 0; num < RobotNum; num++)
			{
				PosSum += q_s(num * RobotDOF + axis, 0);
			}
			X_s(axis, 0) = PosSum / (double)RobotNum;

			double VarSum = 0.0;
			for (int num = 0; num < RobotNum; num++)
			{
				VarSum += pow(q_s(num * RobotDOF + axis, 0) - X_s(axis, 0), 2);
			}
			X_s(axis + TaskSpaceDimension / 2, 0) = VarSum / (double)RobotNum;
		}

		//	update X_m
		X_m = X_m_temp;	//don't put this just after hdWaitForCompletion,
						//or the callback could not be completed yet
						//and the value might be changed unexpected
		/*
		//filter q_s
		if (abs(X_s - X_s_last).max() < 0.01)
			X_s = X_s_last;*/

		if (loopCount < 2)
		{
			X_m_initial = X_m;
			X_s_initial = X_s;
			//median filter
			/*X_mFilterBuf.col(X_mFilterBufFlag) = X_m;
			X_mFilterBufFlag++;
			X_sFilterBuf.col(X_sFilterBufFlag) = X_s;
			X_sFilterBufFlag++;
		}
		else
		{
			X_m = medianFilter(&X_mFilterBuf, &X_m, &X_mFilterBufFlag);
			X_s = medianFilter(&X_sFilterBuf, &X_s, &X_sFilterBufFlag);*/
		}
		/*
		//solve problem: When X_m is negtive over, slaves diverge due to variance command
		for (int axis = 0; axis < TaskSpaceDimension / 2; axis++)
		{
			if (X_s(axis + TaskSpaceDimension / 2, 0) < 1 &&
				X_m(axis + TaskSpaceDimension / 2, 0) < (X_m_initial(axis + TaskSpaceDimension / 2, 0) + X_m_last(axis + TaskSpaceDimension / 2, 0)))
			{
				X_m(axis + TaskSpaceDimension / 2, 0) = X_m_last(axis + TaskSpaceDimension / 2, 0) + X_m_initial(axis + TaskSpaceDimension / 2, 0);
			}
		}*/

		//
		X_m -= X_m_initial;
		X_s -= X_s_initial;
		
		printToTxt(X_m_file, &X_m);
		printToTxt(X_s_file, &X_s);
		printToTxt(q_s_file, &q_s);
		/*
		X_m.t().print("X_m:");
		X_s.t().print("X_s:");
		*/
		//update time derivative of X,q,qDot
		if (loopCount != 0)	//skip at first because last value doesn't exist
		{
			XDot_s = timeDerivative(X_s_last, X_s);
			qDot_m = timeDerivative(q_m_last, q_m);
			qDot_s = timeDerivative(q_s_last, q_s);
			qDoubleDot_s = timeDerivative(qDot_s_last, qDot_s);
		}
		printToTxt(qDot_s_file, &qDot_s);
		/* 
		 * part2:update other parameters
		 */		
		e_m = X_s / alpha - X_m;
		e_s = X_m * alpha - X_s;

		eDot_s = timeDerivative(e_s_last, e_s);
		e_s.t().print("e_s");
		//J_m = ;
		printToTxt(e_s_file, &e_s);

		mat SlaveAveragePosition(RobotDOF, 1, fill::zeros);
		for (int axis = 0; axis < RobotDOF; axis++)
		{
			for (int num = 0; num < RobotNum; num++)
			{
				SlaveAveragePosition(axis,0) += q_s(axis + num * RobotDOF,0);
			}
			SlaveAveragePosition(axis,0) /= (double)RobotNum;
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
				for (int col = 0; col < RobotNum; col++)
				{
					J_s(row, col*RobotDOF + row % RobotDOF) = 1.0 / (double)RobotNum;
				}
			}
			//	variance
			//	v_x	0	0	...
			//	0	v_y	0	...
			//	0	0	v_z	...
			//	v_i = 2/(phi*n)*(q_si-avg_i), i=x,y,z
			else
			{
				for (int col = 0; col < RobotNum; col++)
				{
					J_s(row, col*RobotDOF + row % RobotDOF) 
						= 2 / (double)(/*RobotDOF**/RobotNum)*(q_s(col*RobotDOF + row % RobotDOF, 0) - SlaveAveragePosition(row % RobotDOF, 0));
				}
			}
		}

		//invJ_m = inv( J_m, MatElementTolerence );
		pinvJ_s = pinv( J_s, MatElementTolerence );
		//	secondary task
		//	maximize inner distance
		mat InnerDistancef_a(RobotDOF*RobotNum, 1, fill::zeros);	//partial f_d(2.19)
		mat Distance(RobotNum, RobotNum, fill::zeros);
		for (int row = 0; row < RobotNum; row++)
		{
			for (int col = 0; col < RobotNum; col++)
			{
				for (int axis = 0; axis < RobotDOF; axis++)
				{
					if (row == col)break;
					Distance(row, col) += pow(q_s(col*RobotDOF + axis, 0) - q_s(row*RobotDOF + axis, 0), 2);
				}
				Distance(row, col) = sqrt(Distance(row, col));
			}
		}
		for (int robot = 0; robot < RobotNum; robot++)
		{
			for (int axis = 0; axis < RobotDOF; axis++)
			{
				for (int other = 0; other < RobotNum; other++)
				{
					if (robot != other)
					{
						InnerDistancef_a(robot*RobotDOF + axis, 0) 
							+= (q_s(robot*RobotDOF + axis, 0) - q_s(other*RobotDOF + axis, 0)) / Distance(robot, other);
					}
				}
			}
		}
		InnerDistancef_a *= -2.0 / (double)(RobotNum*(RobotNum - 1));
		//	collision avoidance
		mat CollisionAvoidancef_a(RobotDOF*RobotNum, 1, fill::zeros);

		f_a = -InnerDistancef_a * 10.0 -CollisionAvoidancef_a * 0.0;
		
		JDot_s = timeDerivative(J_s_last, J_s);
		s_m = -inv(J_m) * k_t * e_m + qDot_m;
		s_s = -pinvJ_s * k_t * e_s + qDot_s - ( eye( RobotNum * RobotDOF, RobotNum * RobotDOF ) - pinvJ_s * J_s ) * f_a;
		//Y_s = pinv(JDot_s, MatElementTolerence) * k_t * e_s + pinvJ_s * k_t * eDot_s + qDoubleDot_s;
		//Y_s = timeDerivative(s_s_last, s_s);
		//thetaHatDot_s = -1 * diagmat(Y_s).t() * s_s;
		if (loopCount >= 5)
		{
		//thetaHat_s += thetaHatDot_s * SAMPLING_TIME;
		//u_m = ;
		//u_s = diagmat(Y_s) * thetaHat_s - k_ss * s_s - trans(J_s) * k_p * (-k_t * e_s + XDot_s);
		u_s = (qDoubleDot_s - timeDerivative(s_s_last, s_s)) - k_ss * s_s - trans(J_s) * k_p * J_s * s_s;
		//u saturation
		for (int i = 0; i < u_s.n_rows; i++)
		{
			if (abs(u_s(i, 0)) > 10.0)
			{
				if (u_s(i, 0) > 0)
					u_s(i, 0) = 10.0;
				else
					u_s(i, 0) = -10.0;
			}
		}
		VelocityCommand_s = VelocityCommand_s + u_s * SAMPLING_TIME;	//integrate
		}
		printToTxt(u_s_file, &u_s);
		/*
		//	velocity saturation
		for (int i = 0; i < RobotNum * RobotDOF; i++)
		{
			if (abs(VelocityCommand_s(i, 0)) > 10.0)
				VelocityCommand_s(i, 0) = 10.0;
			if (abs(VelocityCommand_s(i, 0)) < 0.001)
				VelocityCommand_s(i, 0) = 0.0;
		}*/

		/* 
		 * part3:send command to robots
		 */
		
		//	get enough data for median filter in the beginning and send command to keep remote UDP working(for blocking)
		loopCount++;
		if (loopCount < 5)
		{
			if (sendto(s, sFirstSend.c_str(), BUFLEN, 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
			{
				printf("sendto() failed with error code : %d", WSAGetLastError());
				exit(EXIT_FAILURE);
			}
			continue;
		}
		//	to slave
		stringstream ss_to_s;
		string s_to_s;
		for (int num = 0; num < RobotNum; num++)
		{
			for (int axis = 0; axis < RobotDOF; axis++)
			{
				ss_to_s << VelocityCommand_s(num * RobotDOF + axis, 0) << " ";	
			}
		}
		s_to_s = ss_to_s.str();
		if (sendto(s, s_to_s.c_str(), BUFLEN, 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
		{
			printf("sendto() failed with error code : %d", WSAGetLastError());
			exit(EXIT_FAILURE);
		}
		cout << "sendto:" << endl << s_to_s << endl;
		//	clear stringstream
		ss_to_s.str("");
		ss_to_s.clear();
				
		
		//	make fix sampling time
		SYSTEMTIME LoopEnd;
		GetLocalTime(&LoopEnd);
		int ThisLoopTimeMillisecond = ((LoopEnd.wMinute - LoopStart.wMinute) * 60 + (LoopEnd.wSecond - LoopStart.wSecond)) * 1000 + (LoopEnd.wMilliseconds - LoopStart.wMilliseconds);
		int SleepMillisecond = SAMPLING_TIME * 1000 - ThisLoopTimeMillisecond;
		//cout << "sleep:" << SleepMillisecond << endl;
		if (SleepMillisecond > 0)	//make sure the case loop time is not greater than sampling time
			Sleep(SleepMillisecond);
		
		
	}

	//--------end--------	
	hdStopScheduler();
	hdUnschedule(Scheduler_GetCmd1);
	hdUnschedule(Scheduler_GetCmd2);
	hdDisableDevice(DeviceID_1);
	hdDisableDevice(DeviceID_2);
	closesocket(s);
	WSACleanup();
	_fcloseall();

	//system("PAUSE");
	return 0;
}

mat timeDerivative(mat last, mat now)
{
	//double SamplingTime = 0.1;
	return (now - last) / SAMPLING_TIME;
}

HDCallbackCode HDCALLBACK GetCmd1Callback(void *data)
{
	HDdouble command[TaskSpaceDimension / 2];

	//	read command from omni
	hdBeginFrame(DeviceID_1);
	hdGetDoublev(HD_CURRENT_POSITION, command);
	for (int axis = 0; axis < TaskSpaceDimension / 2; axis++)
	{
		X_m_temp(axis, 0) = command[axis];
	}
	
	hdEndFrame(DeviceID_1);
	return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK GetCmd2Callback(void *data)
{
	HDdouble command[TaskSpaceDimension / 2];

	//	read command from omni
	hdBeginFrame(DeviceID_2);
	hdGetDoublev(HD_CURRENT_POSITION, command);
	for (int axis = 0; axis < TaskSpaceDimension / 2; axis++)
	{
		X_m_temp(3 + axis, 0) = command[axis];
	}
	
	hdEndFrame(DeviceID_2);
	return HD_CALLBACK_CONTINUE;
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

void printToTxt(FILE *dst, mat *src)
{
	for (int i = 0; i < src->n_rows; i++)
	{
		fprintf(dst, "%f", src->at(i,0));
		if (i != src->n_rows - 1)
			fprintf(dst, " ");
	}
	fprintf(dst, "\n");
}