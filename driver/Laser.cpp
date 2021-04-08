//crti:2016-07-04,使用hokuyo官方驱动，看是否可以解决激光失效问题
//进程锁未设置
#define EXPORT __declspec(dllexport)
#include "Laser.h"

using namespace qrk;
using namespace std;
using namespace driver;

LaserLib::LaserLib(const std::string& ip, int port):
	working(false),_ip(ip),_port(port)
{
	std::string cfgfile("navigator.ini");
	cfg.read(cfgfile);
	angle_degree_scale = double(cfg.value("laser","angle_degree_scale",90.0));
	std::cout<<"[laser] angle_degree_scale = "<< angle_degree_scale<<std::endl;

	range_max = double(cfg.value("laser","range_max",10.0));
	std::cout<<"[laser] range_max = "<< range_max<<std::endl;
	this->_start();
}

int LaserLib::_start()
{
	// Connects to the sensor//urg.open = connect + GetParameter
    //if (!urg.open(_ip.c_str(), _port, Urg_driver::Ethernet))
	while (!urg.open(_ip.c_str(), _port, Urg_driver::Ethernet))
	{
        cout << "Error,Urg_driver::open(): "
             << _ip.c_str()<< ": " <<_port
			 <<" # "<< urg.what() << endl;
        //return 1;
		//crti todo: neet add while to reconnect laser, if failed
    }
	cout << "Urg_driver::open(): "<< _ip.c_str()
		 << ": " <<_port<< endl;

	//angle_degree_scale = 80;
	// Case where the measurement range (start/end steps) is defined
    urg.set_scanning_parameter(urg.deg2step(-angle_degree_scale), urg.deg2step(+angle_degree_scale), 0);

	//set paras
	//this->max_size = urg.max_data_size();
	max_size = urg.deg2step(angle_degree_scale)*2+1;
	//this->data = new long[this->max_size];
	//memset(data,NULL,sizeof(data));
	angle_increment = urg.step2rad(1);
	angle_min = urg.step2rad(urg.deg2step(-angle_degree_scale));
	angle_max = urg.step2rad(urg.deg2step(+angle_degree_scale));
	ranges.resize(max_size);
	fprintf(stderr,"angle = %.4f:%.4f:%.4f %dbeams \n " , angle_min, angle_increment, angle_max, ranges.size());
	
	//start measurement
	//urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);
	return 0;
}

LaserLib::~LaserLib()
{

}

void LaserLib::update()
{
	urg.start_measurement(Urg_driver::Distance, -1, 0);
	//crti:2016-07-06, need lock?
	long time_stamp = 0;
	if (!urg.get_distance(data, &time_stamp))
	{
		cout << "Urg_driver::get_distance(): " << urg.what() << endl;
		//crti:2016-07-06,需要添加重启动laser
		std::cerr << "laser read fail "<<std::endl;
		return;
	}
	for(int i=0; i < this->max_size; ++i)
	{
		ranges[i] = double(data[i])/1000.0;
		if(ranges[i] < 0.1 || ranges[i] > this->range_max)
			ranges[i] = this->range_max;
	}

}
