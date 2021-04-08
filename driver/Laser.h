//crti:2016-07-04,ʹ��hokuyo�ٷ����������Ƿ���Խ������ʧЧ����
//������δ����
#ifndef _LASERUST_H
#define _LASERUST_H

#include "LaserSrc/Urg_driver.h"
//#include "Connection_information.h"
//#include "math_utilities.h"
#include <iostream>
#include "configfile.h"

using namespace qrk;
using namespace std;

namespace driver
{
class LaserLib
{
public:
	double angle_min;
	double angle_max;
	double angle_increment;
	double range_max;
	int max_size;
	int recv_n;
	std::vector<double> ranges;
	double angle_degree_scale;//crti:2016-07-06,���Ҫд��ini��,-90��90�Ļ�����90
private:
	bool working;
	std::vector<long> data;
	std::string _ip;
	int _port;
	Urg_driver urg;
public:
	LaserLib(const std::string& ip, int port);
	virtual ~LaserLib();
	void update();
private:
	int _start();
	//crti:2016-11-01,add cfg
	middleware::ConfigFile cfg;
};
}


#endif
