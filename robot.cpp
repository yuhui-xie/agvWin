#define EXPORT __declspec(dllexport)
#include "robot.h"


std::string hwid()
{
    int nSize = VMProtectGetCurrentHWID(NULL, 0);
    char *p = new char[nSize];
    VMProtectGetCurrentHWID(p, nSize);
    std::string ret(p);
    delete [] p;
    return ret;
}
void set_licence(const std::string file)
{
    FILE *fp = fopen(file.c_str(), "rb");
    if(fp == NULL)
    {
        std::cerr << "not licenced!" << std::endl;
        exit(-1);
    }
    char buf[4096];
    int size = 0;
    int count = 0;
    while( ( count = fread(buf + size, 1, 1, fp)) != 0 )
    {
        size += count;
    }
    buf[size] = 0;
    VMProtectSetSerialNumber(buf);
    printf("Licence: %s### is set\n", buf);
    fclose(fp);
}
/* serval driver for navigator
 * the GenerelDriver provide the api
 * the handware is based on jiaolong wheelchair, most source is in ./driver
 * the Simulator is for gazebo/jiaolong simulator
 * the Playback is to replay the scene of the logged file
 */

class Hardware : public GenerelDriver
{
private:
    struct _Update_Thread:public BThread
    {
        Hardware* handle;
        _Update_Thread(Hardware* p)
        {
            handle = p;
        }
        ~_Update_Thread()
        {
            kill();
        }
        void run()
        {
            while(true)
            {
                if(handle->laser)
                    handle->laser->update();
                //std::cerr<<"."<<std::endl;
                handle->seq ++;
				//_sleep(10);//crti:2016-05-28,降低cpu使用率
                handle->cond.wakeup();
            }

        }
    } update_thread;

    struct _Odometry_Thread:public BThread
    {
        Hardware* handle;
        _Odometry_Thread(Hardware* p)
        {
            handle = p;
        }
        ~_Odometry_Thread()
        {
            kill();
        }
        void run()
        {
            while(true)
            {
                double v, w, vy;
				int projector;
				int lighttime, lightnum;
				int opcl=0;
				int rifa=0;
                handle->_lock.lock();
                v = handle->_command.v();
                w = handle->_command.w();
				opcl = handle->openclose;
				rifa = handle->risefall;
				//crti:2016-07-05,有关投影仪的
				projector = handle->get_projector();
				lighttime = handle->get_light(); 
				lightnum = handle->get_light2();
				//lighttime = 2;
				//lightnum = 3;
				//std::cout << "lighttime " << lighttime << std::endl;
				//std::cout << "lightnum " << lightnum << std::endl;
				//lightnum = handle->get_lightnum();
				handle->set_projector(-1);//crti:2016-07-05,读取后清空，防止反复发送同样的命令
				handle->set_light(-1);
				handle->set_light2(0);
				//std::cout<<projector;//fortest

				//vy = handle->_command.v2();//crti:2016-05-28,全向的速度控制
				//if(vy > 1.0)vy = 1.0;
				//if(vy <-1.0)vy =-1.0;
                handle->_lock.unlock();
                if(handle->odometry)
                {
					//crti:2016-06-21,添加了全向和差速两种，对应的python文件也不同
					//crti:2016-07-05,添加了有关投影仪的open_or_close_projector
                    handle->odometry->update_diff(v, w, projector,opcl,rifa);
					//crti:2016-05-28,全向的速度控制
					//handle->odometry->update_omni(v, vy, w);
                    handle->_lock.lock();
                    handle->_velocity.set_v( handle->odometry->v() );
                    handle->_velocity.set_w( handle->odometry->w() );
                    handle->_lock.unlock();
                }
				//_sleep(10);//crti:2016-05-28,降低cpu使用率
                //std::cerr<<handle->odometry->x() << " " << handle->odometry->y()<< " "  << handle->odometry->t() <<std::endl;
            }

        }
    } odometry_thread;

    BCond cond;

    BMutex _lock;
    stdmsg::Velocity _command;
	int openclose=0;
	int risefall=0;
    stdmsg::Velocity _velocity;

    //driver::laser *laser;

	driver::LaserLib *laser;
    driver::odometry *odometry;
	driver::odometry **ptrOdomInHardware;

	//crti:2016-07-05
	int open_or_close_projector = -1;
	int open_or_close_light = -1;
	int open_or_close_light2 = 0;

#ifdef WITHOUT_ODOMETRY
    LaserOdometry laser_odometry;
#endif
    unsigned int seq;
    PARAMETER(double, laser_x);
    PARAMETER(double, laser_y);
    PARAMETER(double, laser_theta);
public:
    Hardware(middleware::ConfigFile& cfg):
        update_thread(this),
        odometry_thread(this),
        seq(0)
    {
        std::string port = cfg.value("odometry", "port","com1");
        int  baud = cfg.value("odometry", "baud",19200);
        std::cerr<< "odometry port is "<< port <<" @ " << baud <<std::endl;
        odometry = new driver::odometry( port, baud );
		ptrOdomInHardware = &odometry;
		//Hokuyo
        std::string ip = cfg.value("laser", "ip","192.168.10.10");
        int  port_ = cfg.value("laser", "port", 10940);
        std::cerr<< "laser adress is "<< ip <<" @ " << port_ <<std::endl;
        laser = new driver::LaserLib(ip, port_);
		
#undef helper__
#define helper__(type, x, _default) set_##x ( type(cfg.value("laser_pose", #x, _default) ) );\
    std::cout<<"[laser_pose] "#x<<"\t = ("<<#type<<") "<<x()<<std::endl;
        helper__(double, laser_x, 0.7  );
        helper__(double, laser_y, 0  );
        helper__(double, laser_theta, 0 );

		//crti;2016-07-05,投影仪控制，初始不控制
	    set_projector(-1);

        update_thread.start();
        odometry_thread.start();

        scan();
    }
    ~Hardware(){}
    // multithread
    void set_commad( const stdmsg::Velocity & vel)
    {
        this->_lock.lock();
        this->_command = vel;
        this->_lock.unlock();
    }

	void set_otherstate(const int & openclose, const int & risefall)
	{
		this->_lock.lock();
		this->openclose = openclose;
		this->risefall = risefall;
		this->_lock.unlock();
	}
    // multithread
    stdmsg::Velocity velocity()
    {
        this->_lock.lock();
        stdmsg::Velocity vel( this->_velocity );
        this->_lock.unlock();
        return vel;
    }
    // multithread
    stdmsg::Laser_Scan scan()
    {
        static stdmsg::Laser_Scan scan;

        cond.wait();

        scan.set_seq( seq );
		// For Hokuyo
		std::vector<double> ranges = this->laser->ranges;
		if (scan.config().angle_max() != laser->angle_max || scan.ranges_size() < ranges.size())
		{
			scan.mutable_config()->set_angle_min(laser->angle_min);
			scan.mutable_config()->set_angle_max(laser->angle_max);
			scan.mutable_config()->set_angle_increment(laser->angle_increment);
			scan.mutable_config()->set_range_max(laser->range_max);
			for (int i = scan.ranges_size(); i < ranges.size(); i++)
			{
				scan.mutable_ranges()->Add();
			}
		}

        for(int i = 0; i < ranges.size(); i ++)
        {
            scan.mutable_ranges()->Set(i, ranges[i]);
        }
        // insert the odometry to the scan 
        double steer;
        const void *rdata = this->odometry->reserved();
        if(rdata)
            steer = *((double*)rdata);
        scan.set_steer(steer);
#ifndef WITHOUT_ODOMETRY
        double odom_t = this->odometry->t();
        //odom_t -= steer;

        scan.mutable_robot()->mutable_position()->set_x( this->odometry->x() );
        scan.mutable_robot()->mutable_position()->set_y( this->odometry->y() );
        scan.mutable_robot()->mutable_orentation()->set_yaw( odom_t );

        scan.mutable_pose()->mutable_position()->set_x(this->odometry->x()
                    + cos(odom_t) * _laser_x - sin(odom_t) * _laser_y  );
        scan.mutable_pose()->mutable_position()->set_y(this->odometry->y()
                    + sin(odom_t) * _laser_x + cos(odom_t) * _laser_y  );
        scan.mutable_pose()->mutable_orentation()->set_yaw( odom_t + _laser_theta );
#else
        laser_odometry.update(scan);
        scan.mutable_pose()->mutable_position()->set_x( this->laser_odometry.x() );
        scan.mutable_pose()->mutable_position()->set_y( this->laser_odometry.y() );
        scan.mutable_pose()->mutable_orentation()->set_yaw( this->laser_odometry.t() );

        scan.mutable_robot()->mutable_position()->set_x( this->laser_odometry.x()
                                                         -( cos(_laser_theta) * _laser_x - sin(_laser_theta) * _laser_y)  );
        scan.mutable_robot()->mutable_position()->set_y( this->laser_odometry.y()
                                                         -( sin(_laser_theta) * _laser_x + cos(_laser_theta) * _laser_y)  );
        scan.mutable_robot()->mutable_orentation()->set_yaw( this->laser_odometry.t() - _laser_theta );
#endif
#ifdef DEBUG_HELPPER
        std::cout<<"ranges = [";
        for(int i = 0; i < ranges.size(); i ++)
            std::cout<<scan.ranges(i)<<",";
        std::cout<<"]"<<std::endl;
#endif
        return scan;
    }

	//crti;2016-06-22,加一个返回emergency_stop_flag的函数
	int get_emergency_stop_flag()
	{
		int tmp;
		tmp = (int)this->odometry->emergency_stop_flag();
		return tmp;
	}

	//crti;2016-07-05,投影仪控制
	void set_projector(int open_or_close_projector)
	{
		this->open_or_close_projector = open_or_close_projector;
		//std::cout<<"set_projector:"<<open_or_close_projector<<std::endl;
	}
	int get_projector()
	{
		return this->open_or_close_projector;
	}
	void set_light(int open_or_close_light)
	{
		this->open_or_close_light = open_or_close_light;
		//std::cout<<"set_projector:"<<open_or_close_projector<<std::endl;
	}
	int get_light()
	{
		return this->open_or_close_light;
	}
	void set_light2(int open_or_close_light2)
	{
		this->open_or_close_light2 = open_or_close_light2;
		//std::cout<<"set_projector:"<<open_or_close_projector<<std::endl;
	}
	int get_light2()
	{
		return this->open_or_close_light2;
	}


	//fyf: 2016-12-2, 返回docking_state的函数
	int get_docking_state()
	{
		int tmp;
		tmp = (int)this->odometry->docking_state(); //docking_state()在odometry的helper处定义
		return tmp;
	}
	int get_battery_level()
	{
		return this->odometry->battery_level();
	}
	int get_charge_state()
	{
		return this->odometry->is_charging();
	}
	int get_control_state(){
		return this->odometry->control_sta();
	}
	int get_goal_port(){
		return this->odometry->goal_port();
	}
	int get_set_place_cmd(){
		return this->odometry->set_point();
	}
	int get_record_path_cmd(){
		return this->odometry->record_path();
	}
	int get_initial_pose(){
		return this->odometry->set_now_pos();
	}
	int get_mapping_cmd(){
		return this->odometry->build_map();
	}
};

class Nop : public GenerelDriver
{
    inline void set_commad(const stdmsg::Velocity &vel){};
	inline void set_otherstate(const int &openclose, const int &risefall){};
    inline stdmsg::Velocity velocity(){return stdmsg::Velocity();};
    inline stdmsg::Laser_Scan scan()
    {
        stdmsg::Laser_Scan ret;
        while(1)
#ifdef WIN32
            Sleep(1000);
#else
            sleep(1000000);
#endif
    }

	//crti;2016-06-22,加一个返回emergency_stop_flag的函数
	inline int get_emergency_stop_flag(){return -1;};
	//crti;2016-07-05,投影仪控制
	virtual void set_projector(int open_or_close_projector){};
	virtual void set_light(int open_or_close_light){};
	virtual void set_light2(int open_or_close_light2){};
	virtual int get_projector(){return -1;};
	virtual int get_light(){ return -1; };
	virtual int get_light2(){ return -1; };
	virtual int get_docking_state(){ return 0; };
	virtual int get_battery_level(){ return 0; };
	virtual int get_charge_state(){ return 0; };
	virtual int get_control_state(){ return 0; };
	virtual int get_goal_port(){ return 0; };
};

Robot::Robot(int argc, char** argv) :
    rpc_thread(this),
    nh_thread(this),
    control_mode( AUTONOMOUS )
{
    std::cerr << "hardware ID is: "<<hwid()<<std::endl;
    if(argc == 1)
    {
        std::cout<< "usage:\n" "\t" "-cfg cfgfile. "
                 << "defalut read 'navigator.ini' file in current path"
                 << std::endl
                 << "\t" "-sim. " <<"not open the hardware, but the rpc can be called"<<std::endl
                 <<std::endl;
    }

    /* read the configure file
     * defalut read 'navigator.ini file in the same
     * current path, also it can be specified by the
     * argv, using -cfg option */
    std::string cfgfile("navigator.ini");
    for (int i = 0; i < argc - 1; i++)
        if( strcmp(argv[i],"-cfg") == 0 )
            cfgfile = argv[i+1];
    cfg.read(cfgfile);


    /* default the driver is the hardware
     * sometimes, we want to simulate the hardware
     * so, if the argv has the -sim option, then only
     * the rpc is offered
     */
    bool simulated_driver = false;
    for (int i = 0; i < argc; i++)
        if( strcmp(argv[i],"-sim") == 0 )
            simulated_driver = true;
    //if(simulated_driver)
    //    driver = new Nop;
    //else
    driver = new Hardware(cfg);
		//ptrHardware = &((Hardware*)driver);

    /* framework initialize */
    std::string tmp = cfg.value("node", "bind", "tcp://127.0.0.1:9001");
    std::cout<<"[node] bind = "<< tmp <<std::endl;
    nh = new middleware::Node(tmp);


    /* prarmeter of the topic, including the frequency
     * the frequency is spcified every publish_time scan publish one
     */
    this->velocity_topic = std::string( cfg.value("topic", "velocity", "velocity") );
    this->raw_scan_topic = std::string( cfg.value("topic", "raw_scan", "raw_scan") );
    this->publish_time = int( cfg.value("topic","publish_time",10) );
    this->nh->subscrible(this->velocity_topic, &Robot::set_cmd_manual, this);

    /* address connect to, and then subscribe the topic
     * may be many addresses should be connected, so if
     * the address is splited by the ';', another adrress
     * is connect
     */
    tmp = std::string( cfg.value("node", "connect", "tcp://127.0.0.1:9002") );
    std::cerr<<"[node] connect = " ;
    while( tmp != "" )
    {
        int pos = tmp.find(";");
        std::cerr<<tmp.substr(0, pos)<<"; ";
        nh->connect(tmp.substr(0, pos));

        if (pos != std::string::npos)
            tmp = tmp.substr(pos + 1 );
        else
            tmp = "";
    }
    std::cerr<<std::endl;


    /* the service provided, through the rpc method
     * then the rpc thread is start to check any request
     * is arrived, if yes, reply it
     */
    tmp = std::string( cfg.value("rpc", "address", "tcp://127.0.0.1:9000") );
    std::cout<<"[rpc] address: "<<tmp<<std::endl;
    try{
        rpc.bind(tmp);
        rpc.set("set_mode", &Robot::set_mode, this);
        rpc.set("get_mode", &Robot::get_mode, this);
        rpc.connect(tmp);
    }catch(std::exception &e)
    {
        std::cerr<<e.what()<<std::endl;
        exit(-1);
    }
    rpc_thread.start();
}

Robot::~Robot()
{
    std::cerr<<"node uninitialize!"<<std::endl;
    if(nh)
        delete nh;

    std::cerr<<"drive uninitialize!"<<std::endl;
    if(driver)
        delete driver;
    this->rpc_thread.kill();
}

void Robot::start_ipc()
{
    this->rpc_thread.start();
    this->nh_thread.start();
}

void Robot::set_cmd_manual(const stdmsg::Velocity &vel)
{
    assert(this->driver);
    if(this->control_mode == MANUAL)
    {
        printf("set_cmd_manual called: %.2f %.2f\n", vel.v(), vel.w());
        this->driver->set_commad(vel);
    }
    else
        printf("set_cmd_manual with error state\n");
}

void Robot::set_cmd_autonomous(const stdmsg::Velocity &vel)
{
    assert(this->driver);
    if(this->control_mode == AUTONOMOUS)
        this->driver->set_commad(vel);
}

void Robot::set_otherstate_autonomous(const int &openclose, const int &risefall)
{
	assert(this->driver);
	if (this->control_mode == AUTONOMOUS)
		this->driver->set_otherstate(openclose,risefall);
}

stdmsg::Velocity Robot::odomvelocity()
{
	
	stdmsg::Velocity vel = this->driver->velocity();
	return vel;
}

int Robot::get_battery_level(){
	return driver->get_battery_level();
}
	
int Robot::get_charge_state(){
	return driver->get_charge_state();
	 
}

int Robot::get_control_state(){
	return driver->get_control_state();
}

int Robot::get_goal_port(){
	return driver->get_goal_port();
}


int Robot::get_set_place_cmd(){
	return driver->get_set_place_cmd();
}


int Robot::get_record_path_cmd(){
	return driver->get_record_path_cmd();
}

int Robot::get_initial_pose(){
	return driver->get_initial_pose();
}

int Robot::get_mapping_cmd(){
	return driver->get_mapping_cmd();
}

stdmsg::String Robot::get_mode(const stdmsg::String &mode)
{
    stdmsg::String ret;
    if(this->control_mode == MANUAL)
        ret.set_str("MANUAL");
    else if(this->control_mode == AUTONOMOUS)
        ret.set_str("AUTONOMOS");
    return ret;
}

stdmsg::String Robot::set_mode(const stdmsg::String &mode)
{
    if(mode.str() == "AUTONOMOS")
        this->control_mode = AUTONOMOUS;
    else if(mode.str() == "MANUAL")
        this->control_mode = MANUAL;
    return mode;
}

stdmsg::Laser_Scan Robot::scan( )
{
    return this->driver->scan();
}
