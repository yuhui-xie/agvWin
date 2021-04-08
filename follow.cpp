#include <math.h>
#include <vector>
#include <iostream>

#include <sstream>
#include <image.h>
#include <node.hpp>
#include <stdmsg.hh>
#include <thread.hpp>
#include <queue>
#include <list>

#include <windows.h>
#include <stdio.h>

class Follow
{
protected:
	middleware::Node *nh;
    middleware::RPC rpc;
	middleware::ConfigFile cfg;
private:
	 struct _Rpc_Thread:public BThread
    {
        Follow* handle;
        _Rpc_Thread(Follow* p)
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
        Follow* handle;
        _Node_Thread(Follow* p)
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
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"communication error: "<< e.what() <<std::endl;
            }
        }
    } nh_thread;

    std::string correct_scan_topic;
    double THRESH_MIN_;
    double THRESH_MAX_;
    int Start_index_;
    int End_index_;
    double ANGLE_MIN_;
    double ANGLE_INCREMENT_;
    bool initial_;
    double result_x;
    double result_y;
    bool object_found;
    double laser_offset_x;
    double laser_offset_y;
    double laser_offset_t;
    double safe_distance;
    double robot_size;
public:
    Follow(int argc, char** argv) :
	    rpc_thread(this),
		nh_thread(this),
        //THRESH_MIN_(0.06), 
        //THRESH_MAX_(2.0),
        Start_index_(0),
        End_index_(0),
        //ANGLE_MIN_(-135.0),
        //ANGLE_INCREMENT_(0.5),
        initial_(true),
        result_x(0.0),
        result_y(0.0),
        object_found(false)
    {
		std::string cfgfile("navigator.ini");
		for (int i = 0; i < argc - 1; i++)
			if( strcmp(argv[i],"-cfg") == 0 )
				cfgfile = argv[i+1];
		cfg.read(cfgfile);

        THRESH_MIN_ = double(cfg.value("follow", "thresh_min", "0.06"));
        THRESH_MAX_ = double(cfg.value("follow", "thresh_max", "1.0"));
        laser_offset_x = double(cfg.value("laser_pose", "laser_x", "0.0"));
        laser_offset_y = double(cfg.value("laser_pose", "laser_y", "0.0"));
        laser_offset_t = double(cfg.value("laser_pose", "laser_theta", "0.0"));
        safe_distance = double(cfg.value("follow", "safe_distance", "0.15"));
        robot_size = double(cfg.value("laser_pose", "laser_x", "0.0"));

		//node set and connect, copy from robot.cpp
		std::string tmp = cfg.value("node", "bind_follow", "tcp://127.0.0.1:9004");
		std::cout<<"[node] bind = "<< tmp <<std::endl;
		nh = new middleware::Node(tmp);
		
		tmp = std::string( cfg.value("node", "connect_follow", "tcp://127.0.0.1:9001") );
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

		tmp = std::string( cfg.value("rpc", "address_follow", "tcp://127.0.0.1:8999") );
        rpc.connect(tmp);
        std::cerr<<"[RPC] connect = "<<tmp<<";"<<std::endl;

        correct_scan_topic  = std::string( cfg.value("topic", "scan", "scan") );
        nh->subscrible(correct_scan_topic, &Follow::update, this);

        this->rpc_thread.start();
		this->nh_thread.start();
    }

    int set_goal(const stdmsg::Pose& pos, const stdmsg::Laser_Scan & scan)
    {
        if(pos.orentation().yaw()<0)
        {
            std::cout<<"not set goal"<<std::endl;
            return -1;
        }
        stdmsg::Pose goal_pos;

        double r = sqrt(pos.position().x()*pos.position().x()+pos.position().y()*pos.position().y());
        double k = (r-safe_distance-robot_size)/r;
		//k=1.0;//crti test
		std::cout<<"r="<<r<<";sd="<<safe_distance<<";rs="<<robot_size<<std::endl;
        if(k<0.0){
            std::cout<<"object too close"<<std::endl;
            goal_pos.mutable_position()->set_x(scan.robot().position().x());
            goal_pos.mutable_position()->set_y(scan.robot().position().y());
            goal_pos.mutable_orentation()->set_yaw(scan.robot().orentation().yaw());
        }
        else{
			//这里原来的程序localize.cpp里robot存的是laser，要改一下
			double goal_pos_x =
				scan.robot().position().x() 
				+ k*pos.position().x() * cos(scan.robot().orentation().yaw()) 
				- k*pos.position().y() * sin(scan.robot().orentation().yaw())
				;
			double goal_pos_y =
				scan.robot().position().y() 
				+ k*pos.position().x() * sin(scan.robot().orentation().yaw())
				+ k*pos.position().y() * cos(scan.robot().orentation().yaw())
				;
			double goal_pos_yaw =
				tan(pos.position().y()/pos.position().x())
				;
            goal_pos.mutable_position()->set_x(goal_pos_x);
            goal_pos.mutable_position()->set_y(goal_pos_y);
			goal_pos.mutable_orentation()->set_yaw(goal_pos_yaw);
        }

        rpc.call<stdmsg::Pose,stdmsg::Pose>("set_goal",goal_pos);
		_sleep(50);
		rpc.call<stdmsg::Pose,stdmsg::Pose>("set_goal",goal_pos);

        return 0;
    }

    stdmsg::Pose cal_object_position(const stdmsg::Laser_Scan & scan)
    {
        stdmsg::Pose pos_;
        unsigned int n = scan.ranges_size();;
        //std::cout <<"laserdata.size = " <<n << std::endl;
        if(initial_)
        {
            ANGLE_MIN_ = scan.config().angle_min();
            ANGLE_INCREMENT_ = scan.config().angle_increment();
            THRESH_MAX_ = min(THRESH_MAX_, scan.config().range_max()-0.0000001);
            End_index_ = n;
            initial_ = false;
         }  
         //std::cout << "START" << Start_index_ << std::endl;
         //std::cout << "END" << End_index_ << std::endl;
         std::vector<int> Index;
 
        for (unsigned int i = Start_index_; i < End_index_; i++)
        {
            double r = scan.ranges(i);
            if(r> THRESH_MIN_ && r<THRESH_MAX_)
            {
                Index.push_back(i);
                //std::cout << "Index " << i <<std::endl; 
            }
        }
          
        //std::cout << "Index size:  "<<Index.size() << std::endl;
        int n_Index = Index.size();
        if(n_Index)
        {
            int Start = Index[0];
            int End = Index[n_Index-1];

            //std::cout<< "start: " << Start << std::endl;
            //std::cout<< "end: " << End << std::endl;
           
            double sum_x = 0;
            double sum_y = 0;
            double angle_min = ANGLE_MIN_;
            double angle_increment = ANGLE_INCREMENT_;

            for(unsigned int i=0; i<n_Index; i++)
            {
            int index = Index[i];
            double r = scan.ranges(index);
            double theta = angle_min + index * angle_increment;
            double x = r*cos(theta);
            double y = r*sin(theta);
            //std::cout << "index " <<  index << std::endl;
            //std::cout << "r " << r << std::endl;
            //std::cout << "x " <<x << std::endl;
            //std::cout << "y "<< y << std::endl;
            //std::cout <<"theta "<< theta << std::endl;
            sum_x += x;
            sum_y += y;
            }

            if(Start-20 > 0)  Start_index_ = Start-10;
            if(End + 20 < n)  End_index_ = End + 10;
  
            result_x = sum_x/n_Index;
            result_y = sum_y/n_Index;
            object_found = true;
    
            std::cout << result_x << std::endl;
            std::cout << result_y << std::endl;
            std::cout << "angle_min"<< angle_min << std::endl;
            std::cout << "angle_increment"<< angle_increment << std::endl;

            pos_.mutable_position()->set_x(result_x);
            pos_.mutable_position()->set_y(result_y);
            pos_.mutable_orentation()->set_yaw(0.0);
        }
        else
        {
            object_found = false;
            std::cout << "nothing in the scope" << std::endl;
            pos_.mutable_position()->set_x(0.0);
            pos_.mutable_position()->set_y(0.0);
            pos_.mutable_orentation()->set_yaw(-1.0);
			if(Start_index_>6)Start_index_ -=5;
			if(End_index_<n-6)End_index_ +=5;
        }
        return pos_;
    }

	void update(const stdmsg::Laser_Scan & scan )
    {
		stdmsg::Pose pos;
        pos = this->cal_object_position(scan);		
		set_goal(pos,scan);
        return;
    }

    ~Follow()
    {
        std::cerr<<"node uninitialize!"<<std::endl;
        if(nh) delete nh;
    }
};

int main(int argc, char **argv)
{
    Follow F(argc, argv);
	while(1){_sleep(360000);}
}
