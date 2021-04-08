#include "slam/slam.hpp"
#include "robot.h"

#include <vector>
#include <iostream>
#include <sstream>
//#include <node.hpp>
//#include <stdmsg.hh>
#include <thread.hpp>
#include <math.h>
#include <queue>
#include <list>

/* serval driver for navigator
 * the GenerelDriver provide the api
 * the handware is based on jiaolong wheelchair, most source is in ./driver
 * the Simulator is for gazebo/jiaolong simulator
 * the Playback is to replay the scene of the logged file
 */


#include <shlobj.h>
std::string wchar_t_string(wchar_t *WStr)
{
    size_t len = wcslen(WStr) + 1;
    size_t converted = 0;
    char *CStr;
    CStr=(char*)malloc(len*sizeof(char));
    wcstombs_s(&converted, CStr, len, WStr, _TRUNCATE);
    std::string s=CStr;
    return  s;
}
class App : public Robot
{
private:
    SLAM::Slam *slam;
	int move_mode;//crti:2016-05-28,add 06-21;0=stop,1=nav,2=Velocity,3=follow
	int vel_watchdog;//crti:2016-09-17, add watchdog for vel ctrl, for socket error
public:
    App(int argc, char** argv) :
        slam(NULL),
        Robot(argc, argv)
    {
		vel_watchdog = 0;
		move_mode = 2;
        set_licence("slam.key");
        VMProtectBegin("slam_init");
		//HXW.3.21
		nh->subscrible("set_cmd_vel", &App::set_cmd_vel, this);
		rpc.set("set_movemode", &App::set_move_mode, this);

        VMProtectSerialNumberData sd = {0};
        BOOL res = VMProtectGetSerialNumberData(&sd, sizeof(sd));
        std::string uname = wchar_t_string( sd.wUserName );
        if (hwid() != uname && hwid() != "myhwid")
        {
            std::cerr<<"this licence is not bought for this AGV!\n";
            std::cerr << uname <<std::endl;
            exit(-1);
        }

        /* the main slam method */
        slam = new SLAM::Slam(cfg);
		//crti:2016-05-20,延时，防止初始没有读到里程计信息而使前几帧认为坐标是0，0，0
		_sleep(3000);//1000 = 1s

        try{
            rpc.set("update", &App::update, this);

            rpc.set("current_map", &App::current_map, this);
            rpc.set("maker", &App::maker, this);
        }catch(std::exception &e)
        {
            std::cerr<<e.what()<<std::endl;
            exit(-1);
        }
		stdmsg::String name;
		name.set_str("initial");
		maker(name);
        this->start_ipc();
        VMProtectEnd();
    }


    /* remote process api, it must recevie a serilizable
     * object, and return another serilizable object
     */
	//HXW.3.21
	stdmsg::MoveMode set_move_mode(const stdmsg::MoveMode& mod)
	{
		this->move_mode = mod.move_mode();
		printf("move_mode set to %d (0=stop 1=goal 2=vel 3=follow)\n", move_mode);
		return mod;
	}
	void set_cmd_vel(const stdmsg::Velocity& cmd)
	{
		this->vel_watchdog = 0;//crti:2016-09-17, add watchdog for vel ctrl
		this->set_cmd_autonomous(cmd);
		printf("v1,w1=(%.2f,%.2f)\n", cmd.v(), cmd.w());
	}

    stdmsg::Velocity update(const stdmsg::Laser_Scan & rscan )
    {
        VMProtectBeginVirtualizationLockByKey("slam_update");
        /** if the slam is not null, then slam is performed */
        if(this->slam)
            this->slam->update(rscan);


        stdmsg::Laser_Scan scan;
        stdmsg::Velocity cmd;
        cmd.set_v(0);cmd.set_w(0);

        VMProtectEnd();
        if(scan.seq() % this->publish_time == 0)
        {
            /* publish the status of robot */
            this->nh->publish(this->raw_scan_topic, scan);
            /* verbose the status of robot */
    #ifndef VERBOSE
            fprintf(stdout, "%d odom=(%.2f %.2f %.2f %.2f) ", rscan.seq(),
                    rscan.robot().position().x(),
                    rscan.robot().position().y(),
                    rscan.robot().orentation().yaw(),
                    rscan.steer() );
            fprintf(stdout, "cmd=(%.2f,%.2f)\n",cmd.v(), cmd.w() );
    #endif
        }
        /* return the command velocity */
        return cmd;
    }
    stdmsg::String maker(const stdmsg::String& name)
    {
        std::cout<<"maker called with name = "<<name.str().c_str()<<std::endl;
        std::string sname = name.str().c_str();
        if(this->slam && sname != "end")
            this->slam->maker(sname);
        else if(this->slam)
            this->slam->generate_map("map.png","mapnav.png");
        return name;
    }
    stdmsg::LaserList current_map(const stdmsg::LaserList& input)
    {
        stdmsg::LaserList ret;
        if(!this->slam)
            return ret;

        ret = this->slam->scans(false);
        return ret;
    }

    ~App()
    {
        std::cerr<<"slam uninitialize!"<<std::endl;
        if(slam)
            delete slam;
    }

    void run()
    {
        while(1)
        {
            /* get scan from the driver
             *  it is a block method, and only return the
             *  lateset scan
             */
            stdmsg::Laser_Scan scan = this->scan();
            stdmsg::Velocity cmd = this->update(scan);
			_sleep(1);
        }
    }

};

int main(int argc, char **argv)
{
	App nav(argc, argv);
	nav.run();
}
