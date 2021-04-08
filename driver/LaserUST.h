#ifndef _LASERUST_H
#define _LASERUST_H

#include <iostream>
#include <windows.h>
#include <string>
#include <vector>
#include <thread.hpp>
#include "net/sockets.hpp"

#ifdef WIN32
#ifndef EXPORT
#define EXPORT __declspec(dllimport)
#endif
#else
#define EXPORT
#endif

// LaserLib

enum
{
	TimeOut = 800,
	LineLength = 16 + 64 + 1 + 1 + 1 + 3,
};

typedef struct
{
	enum
	{
		MODL = 0,		//!< Sensor Model
		DMIN,			//!< Min detection range [mm]
		DMAX,			//!< Man detection range [mm]
		ARES,			//!< Angular resolution (division of 360degree)
		AMIN,			//!< Min Measurement step
		AMAX,			//!< Max Measurement step
		AFRT,			//!< Front Step 
		SCAN,			//!< Standard scan speed
	};
	std::string model;		//!< Obtained Sensor Model,  MODL
	long distance_min;		//!< Obtained DMIN 
	long distance_max;		//!< Obtained DMAX 
	int area_total;		//!< Obtained ARES 
	int area_min;			//!< Obtained AMIN 
	int area_max;			//!< Obtained AMAX 
	int area_front;		//!< Obtained AFRT 
	int scan_rpm;			//!< Obtained SCAN 

	int first;			//!< Scan Start position
	int last;			//!< Scan end position
	int max_size;			//!< Max. size of data
	long last_timestamp; //!< Time stamp of the last obtained data
}urgState;
namespace driver
{
class EXPORT LaserLib : private TD::Socket
{
//	DECLARE_DYNAMIC(LaserLib)

public:
	//HANDLE HComm;
	//std::string ErrorMessage;
	urgState state;
	double angle_min;
	double angle_max;
	double angle_increment;
	double range_max;
	int max_size;
	int recv_n;
	std::vector<double> ranges;

public:
	LaserLib(const std::string& ip, int port);
	virtual ~LaserLib();
	void update();

	int comConnect(const std::string& ip, int port);
	void comDisconnect();
	int comSend(char *data, int size);
	int comRecv(char *data, int max_size, int timeout);

	int urgSendTag(const char *tag);                                         // Send data(Commands) to URG 
	int urgReadLine(char *buffer);                                           // Read data (Reply) from URG until the termination
	int urgSendMessage(const char *command, int timeout, int *recv_n);       // Send data (Commands) to URG and wait for reply
	int urgGetParameters(urgState *state);                                   // Read URG parameters
	int urgConnect(urgState *state, const char *port, const long baud_rate); // Process to connect with URG
	void urgDisconnect();                                                    // Disconnect URG 
	int urgCaptureByGD(const urgState *state);                               // Data read using GD-Command
	int urgCaptureByMD(const urgState *state, int capture_times);            // Data read using MD-Command
	long urgDecode(const char *data, int data_byte);                         // Decode 6 bit data from URG 
	int urgAddReceiveData(const char buffer[], long data[], int *filled);    // Receive URG distance data 
	int urgReceiveData(urgState* state, long data[], size_t max_size);       // Receive URG data

	//crti:2016-06-28
	void restart1();
	void restart2();
	void restart3();



private:
	BMutex _lock;
	void _start();
	bool working;
	long *data;
	std::string _ip;
	int _port;

	int test;
	
	
//protected:
	//DECLARE_MESSAGE_MAP()
};
}

#endif