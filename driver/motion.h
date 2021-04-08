#ifndef MOTIONH
#define MOTIONH

//#include <stdmsg.hh>
//#include <node.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <thread.hpp>

namespace driver
{


class EXPORT odometry
{
protected:
	//middleware::RPC gp;
	//middleware::ConfigFile cfg;
private:
	/*struct _Rpc_Thread :public BThread
	{
		odometry* handle;
		_Rpc_Thread(odometry* p)
		{
			handle = p;
		}
		~_Rpc_Thread()
		{
			kill();
		}
		void run()
		{
			try{
				while (true)
				{
					handle->gp.run();
				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "communication error: " << e.what() << std::endl;
			}
		}
	} rpc_thread;*/
public:
    void update_diff(double cmd_v, double cmd_w, int projector,int lighttime, int lightnum);
	void update_omni(double cmd_v, double cmd_vy, double cmd_w);
	odometry(const std::string &port, int baud);
    ~odometry();
    void reserved(void* data, int len);
    inline const void* reserved()
    {
        _lock.lock();
        void *ret = reserved_data;
        _lock.unlock();
        return ret;
    };
    inline int reserved_size()
    {
        _lock.lock();
        int ret = reserved_len;
        _lock.unlock();
        return ret;
    };
private:
	#define helper__(type, x) private:type _##x;public:inline type x(){_lock.lock();type tmp = _##x;_lock.unlock();return tmp;}
    helper__(double, x);
    helper__(double, y);
    helper__(double, t);
    helper__(double, v);
	helper__(double, vy);
    helper__(double, w);
	helper__(double, emergency_stop_flag);//crti:2016-06-22, add emergency_stop_flag
	helper__(double, docking_state); //fyf: 2016-12-2, add _docking_state
	helper__(int, battery_level);//fyf: 2016-12-24
	helper__(int, is_charging);//fyf: 2016-12-24
	helper__(int, control_sta);//xbh
	helper__(int, goal_port);//xbh
	helper__(int, height);//xbh
	helper__(int, weight);//xbh
	helper__(int, record_path);//xbh
	helper__(int, set_point);//xbh
	helper__(int, build_map);//xbh20210326
	helper__(int, set_now_pos);//xbh20210326
	#undef helper__
private:
    BMutex _lock;
    //serial::Serial *_serial;
    std::string _port;
    int _baud;
    void * reserved_data;
    int reserved_len;
};



}
#endif
