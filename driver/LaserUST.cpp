// LaserLib.cpp : 实现文件
#define EXPORT __declspec(dllexport)
#include "LaserUST.h"
#include <sstream>
using namespace driver;

// LaserLib
LaserLib::LaserLib(const std::string& ip, int port):
	working(false),_ip(ip),_port(port)
{
	test=0;

	range_max = 10.0;
	this->_start();

	
}

void LaserLib::_start()
{
	//if(working)
		//return;
	
	int ret = this->urgConnect(&state,_ip.c_str(),_port);
	//std::cout<<"test"<<std::endl;
	if (ret<0)
	{
		printf("connect failed");
		this->urgDisconnect();
	}
	else
	{
		this->max_size = state.max_size;
		this->data = new long[this->max_size];
		memset(data,NULL,sizeof(data));
		angle_increment = 0.25 * 3.141592653 /180;
		angle_min = state.first * angle_increment - 135 * 3.1415926 /180;
		angle_max = state.last * angle_increment -135 * 3.1415926 /180;
		//angle_min = state.first * angle_increment - 0 * 3.1415926 /180;
		//angle_max = state.last * angle_increment -0 * 3.1415926 /180;
		ranges.resize(max_size);

		fprintf(stderr,"angle = %.4f:%.4f:%.4f %dbeams \n " , angle_min, angle_increment, angle_max, ranges.size());
		fprintf(stderr,"urg is working now ");

		int recv_n = 0;
		this->urgSendMessage("BM", 1000, &recv_n);   //start measurement 
		this->working = true ;

		//for(int i=0;i<10;++i)
			//this->update();
	}

}

//crti:2016-06-28
void LaserLib::restart1()
{
	this->clearBuffer();
	std::cout<<"restart1,RB"<<std::endl;
	SYSTEMTIME st = {0};
	GetLocalTime(&st);
	printf("%d-%02d-%02d %02d:%02d:%02d\n",st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);

	urgSendMessage("RB", TimeOut, &recv_n);
	urgSendMessage("RB", TimeOut, &recv_n);
	_sleep(10000);
	this->urgSendMessage("BM", 1000, &recv_n);
	this->working = false;
	_start();
}
//crti:2016-06-28
void LaserLib::restart2()
{
	this->clearBuffer();
	std::cout<<"restart2,QT,BM"<<std::endl;
	SYSTEMTIME st = {0};
	GetLocalTime(&st);
	printf("%d-%02d-%02d %02d:%02d:%02d\n",st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);

	urgSendMessage("QT", TimeOut, &recv_n);
	_sleep(5000);
	urgSendMessage("BM", TimeOut, &recv_n);
}

//crti:2016-06-29
void LaserLib::restart3()
{
	if(test==1)
		return;

	std::cout<<"restart3"<<std::endl;
	SYSTEMTIME st = {0};
	GetLocalTime(&st);
	printf("%d-%02d-%02d %02d:%02d:%02d\n",st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);

	char tmp[100];
	sprintf(tmp,"%d-%02d-%02d-%02d-%02d-%02d.txt",st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);
	FILE* fp;
	fp = fopen(tmp,"w+");
	fclose(fp);

	urgSendMessage("RB\n", TimeOut, &recv_n);
	urgSendMessage("RB\n", TimeOut, &recv_n);

	socket_restart();
	std::cout<<"socket_restart"<<std::endl;
	_sleep(15000);
	this->working = false;
	this->_start();

	test=1;
}



LaserLib::~LaserLib()
{
	urgSendMessage("QT", TimeOut, &recv_n);
	urgDisconnect();
	
	//CloseHandle(HComm);
	
	delete[] data;
	
	exit(0);
}

void LaserLib::update()
{
	
	this->_lock.lock();
	int cnt=0;//crti:2016-06-28
	try
	{	
		this->urgCaptureByGD(&state);
		int r_n = this->urgReceiveData(&state,data,max_size);
		if (r_n>0)
		{
				//printf("r_n,max_size is %d%d\n",r_n,max_size);
				for(int i=0;i<ranges.size();i++)
				{
					ranges[i] = double(data[i])/1000.0;
					//ranges[i] = double(data[ranges.size()-i-1])/1000.0;

					////crti:2016-06-28,激光失效检测
					//if(ranges[i] < 0.2 || ranges[i] > this->range_max)
					//	++cnt;
					////std::cout<<cnt<<std::endl;
					////if(ranges[i] < this->range_max)

					if (ranges[i] < 0.1 || ranges[i] > this->range_max)
						ranges[i] = this->range_max;
					//crti:2016-06-28,把激光角度范围放到一个for循环里
					if(i<200)
						ranges[i] = this->range_max;
					if(i>ranges.size()-200)
						ranges[i] = this->range_max;
					

				}
				/*for (int i=0;i<30;i++)
				{
					ranges[i] = this->range_max;
				}
				for(int i=ranges.size()-30;i<ranges.size();i++)
				{
					ranges[i] = this->range_max;
				}*/
		}
	}
	catch(...)
	{
		//std::cout<<"f";
		std::cerr << "laser read fail "<<std::endl;
		//this->working = false;
		//_start();
	}
	////crti:2016-06-28,激光失效检测
	//if(cnt>800)
	//	this->restart3();
	this->_lock.unlock();
}





// LaserLib 消息处理程序
int LaserLib::comConnect(const std::string& ip, int port)
{
    try
    {
        this->connect(ip.c_str(), port);
    }catch(int &e)
    {
        std::cerr << "connect exception: "  <<_ip<<":"<<_port<< ", try to reconnect ..." <<std::endl;
        return -1;
    }
    return 0;
}

void LaserLib::comDisconnect()
{
}

int LaserLib::comSend(char *data, int size)
{
	int ret;
	ret = _send(data,size);
	return ret;
}

int LaserLib::comRecv(char *data, int max_size, int timeout)
{
	int ret;
	ret = recv(data,max_size);

	return ret;
}

int LaserLib::urgSendTag(const char *tag)
{
	char send_message[LineLength];
	sprintf_s(send_message, "%s\n", tag);
	int send_size = strlen(send_message);
	comSend(send_message, send_size);
	return send_size;
}

int LaserLib::urgReadLine(char *buffer)
{
	int i;
	for (i = 0; i < LineLength - 1; i++) 
	{
		char recv_ch;
		int n = comRecv(&recv_ch, 1, TimeOut);
		if (n <= 0) 
		{
			if (i == 0) 
			{
				return -1; // timeout
			}
			break;
		}
		if ((recv_ch == '\r') || (recv_ch == '\n')) 
		{
			break;
		}
		buffer[i] = recv_ch;
	}
	buffer[i] = '\0';

	return i;
}

int LaserLib::urgSendMessage(const char *command, int timeout, int *recv_n)
{
	int send_size = urgSendTag(command);
	int recv_size = send_size + 2 + 1 + 2;
	char buffer[LineLength];

	int n = comRecv(buffer, recv_size, timeout);
	*recv_n = n;

	if (n < recv_size) 
	{
		// When received size not matched
		return -1;
	}

	if (strncmp(buffer, command, send_size - 1)) 
	{
		// When command not matched
		return -1;
	}

	// !!! If possible do calculate check-sum to verify data
	// Convert the received data to Hex and return data.
	char reply_str[3] = "00";
	reply_str[0] = buffer[send_size];
	reply_str[1] = buffer[send_size + 1];
	return strtol(reply_str, NULL, 16);
}

int LaserLib::urgGetParameters(urgState *state)
{
	// Parameter read and prcessing (use)
	urgSendTag("PP");
	char buffer[LineLength] = { 0 };
	int line_index = 0;
	enum 
	{
		TagReply = 0,
		DataReply,
		Other,
	};
	int line_length;
	for (; (line_length = urgReadLine(buffer)) > 0; line_index++)
	{
		if (line_index == Other + urgState::MODL)
		{
			buffer[line_length - 2] = '\0';
			state->model = &buffer[5];
		}
		else if (line_index == Other + urgState::DMIN)
		{
			state->distance_min = atoi(&buffer[5]);
		}
		else if (line_index == Other + urgState::DMAX)
		{
			state->distance_max = atoi(&buffer[5]);
		}
		else if (line_index == Other + urgState::ARES)
		{
			state->area_total = atoi(&buffer[5]);
		}
		else if (line_index == Other + urgState::AMIN)
		{
			state->area_min = atoi(&buffer[5]);
			state->first = state->area_min;
		}
		else if (line_index == Other + urgState::AMAX)
		{
			state->area_max = atoi(&buffer[5]);
			state->last = state->area_max;
		}
		else if (line_index == Other + urgState::AFRT)
		{
			state->area_front = atoi(&buffer[5]);
		}
		else if (line_index == Other + urgState::SCAN)
		{
			state->scan_rpm = atoi(&buffer[5]);
		}
	}
	if (line_index <= Other + urgState::SCAN)
	{
		return -1;
	}
	// Caluclate size of the data if necessary
	state->max_size = state->area_max + 1;

	return 0;
}

int LaserLib::urgConnect(urgState *state, const char *port, const long baud_rate)
{
	if (comConnect(_ip, _port) < 0) 
	{
		std::cerr<<"Cannot connect laser."<<std::endl;
		return -1;
	}

	
	if (urgGetParameters(state) < 0) 
	{
		std::cerr<<"PP command fail."<<std::endl;
		return -2;
	}
	state->last_timestamp = 0;

	// URG is detected
	return 0;
}

void LaserLib::urgDisconnect()
{
	comDisconnect();
}

int LaserLib::urgCaptureByGD(const urgState *state)
{
	char send_message[LineLength];
	sprintf_s(send_message, "GD%04d%04d%02d", state->first, state->last, 0);
	return urgSendTag(send_message);
}

int LaserLib::urgCaptureByMD(const urgState *state, int capture_times)
{
	char send_message[LineLength];
	sprintf_s(send_message, "MD%04d%04d%02d%01d%02d", state->first, state->last, 0, 0, capture_times);
	return urgSendTag(send_message);
}

long LaserLib::urgDecode(const char *data, int data_byte)
{
	long value = 0;
	for (int i = 0; i < data_byte; i++) 
	{
		value <<= 6;
		value &= ~0x3f;
		value |= data[i] - 0x30;
	}
	return value;
}

int LaserLib::urgAddReceiveData(const char buffer[], long data[], int *filled)
{
	static int remain_byte = 0;
	static char remain_data[3];
	const int data_byte = 3;

	const char* pre_p = buffer;
	const char* p = pre_p;

	if (remain_byte > 0) 
	{
		memmove(&remain_data[remain_byte], buffer, data_byte - remain_byte);
		data[*filled] = urgDecode(remain_data, data_byte);
		(*filled)++;
		pre_p = &buffer[data_byte - remain_byte];
		p = pre_p;
		remain_byte = 0;
	}

	do 
	{
		p++;
		if ((p - pre_p) >= static_cast<int>(data_byte)) 
		{
			data[*filled] = urgDecode(pre_p, data_byte);
			++(*filled);
			pre_p = p;
		}
	} while (*p != '\0');
	remain_byte = p - pre_p;
	memmove(remain_data, pre_p, remain_byte);

	return 0;
}

int LaserLib::urgReceiveData(urgState* state, long data[], size_t max_size)
{
	int filled = 0;
	// Fill the positions upto first or min by 19 (non-measurement range)
	for (int i = state->first - 1; i >= 0; --i) 
	{
		data[filled++] = 19;
	}

	char message_type = 'M';
	char buffer[LineLength];
	int line_length;
	for (int i = 0; (line_length = urgReadLine(buffer)) >= 0; ++i) 
	{
		// Verify the checksum
		if ((i >= 6) && (line_length == 0)) 
		{
			// End of data receive
			for (size_t i = filled; i < max_size; ++i) 
			{
				// Fill the position upto data end by 19 (non-measurement range)
				data[filled++] = 19;
			}
			return filled;
		}
		else if (i == 0) 
		{
			// Judge the message (Command) by first letter of receive data
			if ((buffer[0] != 'M') && (buffer[0] != 'G')) 
			{
				return -1;
			}
			message_type = buffer[0];
		}
		else if (!strncmp(buffer, "99b", 3)) 
		{
			// Detect "99b" and assume [time-stamp] and [data] to follow
			i = 4;
		}
		else if ((i == 1) && (message_type == 'G')) 
		{
			i = 4;
		}
		else if (i == 4) 
		{
			// "99b" Fixed
			if (strncmp(buffer, "99b", 3)) 
			{
				return -1;
			}
		}
		else if (i == 5) 
		{
			state->last_timestamp = urgDecode(buffer, 4);
		}
		else if (i >= 6) 
		{
			// Received Data
			if (line_length > (64 + 1)) 
			{
				line_length = (64 + 1);
			}
			buffer[line_length - 1] = '\0';
			int ret = urgAddReceiveData(buffer, data, &filled);
			if (ret < 0) 
			{
				return ret;
			}
		}
	}
	return -1;
}

