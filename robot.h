#ifndef ROBOT_H
#define ROBOT_H
#include <node.hpp>
#include <stdmsg.hh>
#include <thread.hpp>

#include "driver\motion.h"

#include "driver\Laser.h"
#include "driver\LaserUST.h"
//#include "driver\LaserSick\laser.h"

#include <vector>
#include <iostream>
#include <sstream>
#include <math.h>
#include <queue>
#include <list>
#include "VMProtectSDK.h"
std::string hwid();
void set_licence(const std::string file = "licence.key");
#define EXPORT
#ifdef WIN32
#ifndef EXPORT
#define EXPORT __declspec(dllimport)
#endif
#else
#define EXPORT
#endif


/* serval driver for navigator
 * the GenerelDriver provide the api
 * the handware is based on jiaolong wheelchair, most source is in ./driver
 * the Simulator is for gazebo/jiaolong simulator
 * the Playback is to replay the scene of the logged file
 */

class GenerelDriver
{
public:
    virtual void set_commad( const stdmsg::Velocity & vel) = 0;//控制前后左右
	virtual void set_otherstate(const int & openclose, const int & risefall) = 0;//控制升降杆，支腿
    virtual stdmsg::Velocity velocity() = 0;
    virtual stdmsg::Laser_Scan scan() = 0;

	//crti;2016-06-22,加一个返回emergency_stop_flag的函数
	virtual int get_emergency_stop_flag() = 0;
	//crti;2016-07-05,投影仪控制
	virtual void set_projector(int open_or_close_projector) = 0;
	virtual int get_projector() = 0;

	virtual void set_light(int open_or_close_light) = 0;
	virtual int get_light() = 0;

	virtual void set_light2(int open_or_close_light) = 0;
	virtual int get_light2() = 0;

	//virtual void set_light2(int open_or_close_light2) = 0;
	//virtual int get_light2() = 0;


	//fyf, 2016-12-2, 增加返回docking_state的函数
	virtual int get_docking_state() = 0;
	virtual int get_battery_level() = 0;
	virtual int get_charge_state() = 0;
	virtual int get_control_state() = 0;
	virtual int get_goal_port() = 0;
	virtual int get_set_place_cmd() = 0;
	virtual int get_record_path_cmd() = 0;
	virtual int get_initial_pose() = 0;
	virtual int get_mapping_cmd() = 0;
};

enum Control_Mode
{
    MANUAL,
    AUTONOMOUS
};

class EXPORT Robot
{
protected:
    middleware::Node *nh;
    middleware::RPC rpc;

    middleware::ConfigFile cfg;
    int publish_time;
    std::string raw_scan_topic;
private:
    struct _Rpc_Thread:public BThread
    {
        Robot* handle;
        _Rpc_Thread(Robot* p)
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
                while(true)
                {
                    handle->rpc.run();
					_sleep(1);//crti:2016-05-28,降低cpu使用率
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"communication error: "<< e.what() <<std::endl;
            }
        }
    } rpc_thread;

    struct _Node_Thread:public BThread
    {
        Robot* handle;
        _Node_Thread(Robot* p)
        {
            handle = p;
        }
        ~_Node_Thread()
        {
            kill();
        }
        void run()
        {
            try{
                while(true)
                {
                    handle->nh->run();
					_sleep(1);//crti:2016-05-28,降低cpu使用率
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"communication error: "<< e.what() <<std::endl;
            }
        }
    } nh_thread;
    GenerelDriver *driver;
    std::string velocity_topic;

    Control_Mode control_mode;
    stdmsg::Velocity manual_vel;

	//GenerelDriver** ptrHardware;

public:
    Robot(int argc, char** argv) ;
    /* remote process api, it must recevie a serilizable
     * object, and return another serilizable object
     */
	
	void start_ipc();
    stdmsg::Laser_Scan scan( );
    stdmsg::String set_mode(const stdmsg::String& mode);
    stdmsg::String get_mode(const stdmsg::String& mode);
    void set_cmd_manual(const stdmsg::Velocity & vel);
    void set_cmd_autonomous(const stdmsg::Velocity & vel);
	void set_otherstate_autonomous(const int & openclose, const int & risefall);
    ~Robot();

	//crti;2016-06-22,加一个返回emergency_stop_flag的函数
	int get_emergency_stop_flag()
	{
		return this->driver->get_emergency_stop_flag();
	}

	int get_docking_state()
	{
		return this->driver->get_docking_state();
	}
	int get_battery_level();
	int get_charge_state();
	int get_control_state();
	int get_goal_port();
	int get_set_place_cmd();
	int get_record_path_cmd();
	int get_initial_pose();
	int get_mapping_cmd();

	stdmsg::Velocity odomvelocity();

    //virtual void run();

	//crti;2016-07-05,投影仪控制
	void set_projector_flag(int open_or_close_projector)
	{
		this->driver->set_projector(open_or_close_projector);
	}
	void set_light_flag(int open_or_close_light)
	{
		this->driver->set_light(open_or_close_light);
	}
	void set_light2_flag(int open_or_close_light2)
	{
		this->driver->set_light2(open_or_close_light2);
	}
	
};


#endif
