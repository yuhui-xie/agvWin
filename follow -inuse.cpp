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
    double thresh_max;
    double thresh_min;
	double thresh_degree;
    int Start_index_;
    int End_index_;
	int Start_index_Begin;
    int End_index_Begin;
    double ANGLE_MIN_;
    double ANGLE_INCREMENT_;
    bool initial_;
    double result_x;
    double result_y;
    bool object_found;
    double safe_distance;
    double robot_size;

	PARAMETER(double, stop_delta_distance);
	PARAMETER(double, stop_delta_theta);
	PARAMETER(double, nav_v);
	PARAMETER(double, nav_w);
public:
    Follow(int argc, char** argv) :
	    rpc_thread(this),
		nh_thread(this),
        //thresh_min(0.06), 
        //thresh_max(2.0),
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

		#undef helper__
        #define helper__(type, x, _default) set_##x ( type(cfg.value("local_plan", #x, _default) ) );\
        std::cout<<"[follow] "#x<<"\t = ("<<#type<<") "<<x()<<std::endl;
        helper__(double, stop_delta_distance, 0.1);//m
		helper__(double, stop_delta_theta, 10);//°
		helper__(double, nav_v, 0.3);//m/s
		helper__(double, nav_w, 0.25);//rad/s

		#undef helper__
        #define helper__(type, x, _default) x = type(cfg.value("follow", #x, _default));\
        std::cout<<"[follow] "#x<<"\t = ("<<#type<<") "<<x<<std::endl;
		helper__(double,thresh_min,0.06);
		helper__(double,thresh_max,1.0);
		helper__(double,safe_distance,0.15);
		helper__(double,thresh_degree,40);

        robot_size = double(cfg.value("laser_pose", "laser_x", "0.0"));

		//node set and connect, copy from robot.cpp
		std::string tmp = cfg.value("node", "bind_follow", "tcp://127.0.0.1:9005");
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

		tmp = std::string( cfg.value("rpc", "address_follow", "tcp://127.0.0.1:9000") );
        rpc.connect(tmp);
        std::cerr<<"[RPC] connect = "<<tmp<<";"<<std::endl;

        correct_scan_topic  = std::string( cfg.value("topic", "scan", "scan") );
        nh->subscrible(correct_scan_topic, &Follow::update, this);

        this->rpc_thread.start();
		this->nh_thread.start();
    }

	//crti:2016-08-18,检测前方一定范围内有没有障碍物
	//和localizer.cpp里的一样，参数不同
	int find_obstacle_forward(const stdmsg::Laser_Scan & rscan)
	{
		int cnt = 0;
		double min_distance,max_distance;//距离范围，单位米
		int min_laser,max_laser;

		/*
		//crti:2016-08-18,换成直接用follow里的public变量，cal_object_position里面会初始化这些变量，然后set_vel才调用这个函数
		min_distance = 0.4;
		max_distance = 0.9;
		double angle = 60.0;//角度范围，60.0表示-30度到+30度
		//crti:2016-08-16,原来是按照1080线算的，现在变成动态的了
		min_laser = (int)((rscan.ranges().size()-1)/2+1 - angle*0.5 * 4);
	    if (min_laser < 0){ min_laser = 0; std::cout << "ob,out range,<" << std::endl; }
	    max_laser = (int)((rscan.ranges().size()-1)/2+1 + angle*0.5 * 4);
	    if (max_laser > rscan.ranges().size()-1){ max_laser = rscan.ranges().size()-1; std::cout << "ob,out range,>" << std::endl; }
		*/
		min_distance = this->thresh_min;
		max_distance = this->safe_distance;
		min_laser = this->Start_index_Begin;
		max_laser = this->End_index_Begin;
		for(int i=min_laser;i<max_laser;++i)
		{
			double r = rscan.ranges(i);
			if(r>min_distance && r<max_distance)
			{
				++cnt;
			}
		}

		if(cnt>24)//凭感觉给的，60度一共240个点，50个点在范围内就认为有物体
		{
			//std::cout<<"1";
			return 1;
		}
		else
		{
			//std::cout<<"0";
			return 0;
		}
	}

    stdmsg::Pose cal_object_position(const stdmsg::Laser_Scan & scan)
    {
        stdmsg::Pose pos_;
        unsigned int n = scan.ranges_size();
        std::cout <<"laserdata.size = " <<n << std::endl;
        if(initial_)
        {
            ANGLE_MIN_ = scan.config().angle_min();
            ANGLE_INCREMENT_ = scan.config().angle_increment();
            thresh_max = min(thresh_max, scan.config().range_max()-0.0000001);
			int thresh_degree_index = (int)(thresh_degree * (3.1415926 / 180) / scan.config().angle_increment());
			Start_index_Begin = (int)max(0,min((n-1)/2+1 - thresh_degree_index, (n-1)/2+1 + thresh_degree_index));
			End_index_Begin = (int)min(n,max((n-1)/2+1 - thresh_degree_index, (n-1)/2+1 + thresh_degree_index));
			Start_index_ = Start_index_Begin;
            End_index_ = End_index_Begin;
            initial_ = false;
         }
		/*std::cout << "ANGLE_INCREMENT_" << ANGLE_INCREMENT_ << std::endl;
		 std::cout << "Start_index_Begin" << Start_index_Begin << std::endl;
         std::cout << "End_index_Begin" << End_index_Begin << std::endl;
         std::cout << "START" << Start_index_ << std::endl;
         std::cout << "END" << End_index_ << std::endl;*/
         std::vector<int> Index;
 
        for (unsigned int i = Start_index_; i < End_index_; i++)
        {
            double r = scan.ranges(i);
            if(r> thresh_min && r<thresh_max)
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

            if(Start-11 > Start_index_Begin)  Start_index_ = Start-10;
            if(End + 11 < End_index_Begin)  End_index_ = End + 10;
  
            result_x = sum_x/n_Index;
            result_y = sum_y/n_Index;
            object_found = true;
    
            //std::cout << result_x << std::endl;
            //std::cout << result_y << std::endl;
            //std::cout << "angle_min"<< angle_min << std::endl;
            //std::cout << "angle_increment"<< angle_increment << std::endl;

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
			if(Start_index_ > Start_index_Begin+6) Start_index_ -=5;
			if(End_index_ < End_index_Begin-6) End_index_ +=5;
        }
        return pos_;
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
				atan(pos.position().y()/pos.position().x())
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

	void set_vel(const stdmsg::Pose& pos, const stdmsg::Laser_Scan & scan)
	{
		stdmsg::Velocity cmd;
		double v = 0.0, w = 0.0;

		if(pos.orentation().yaw()<0)
        {
            std::cout<<"set velocity = 0"<<std::endl;
			v = 0;
			w = 0;
        }
		else
		{
			double error_theta = atan(pos.position().y()/pos.position().x());
			double error_distance = sqrt(pos.position().x()*pos.position().x()+pos.position().y()*pos.position().y()) - safe_distance;

			while(error_theta> 3.141592653) error_theta -= 3.141592653*2;
			while(error_theta<-3.141592653) error_theta += 3.141592653*2;

			if(error_distance > 0.5 && error_distance < 2)
				v = nav_v();
		    else if (error_distance > stop_delta_distance() && error_distance < 0.5)
				v = nav_v()/1.5;
		    else if (error_distance<stop_delta_distance())
				v = 0.0;
		    if (fabs(error_theta) > stop_delta_theta()*3.141592653 / 180)
				v = 0.0;
			w = error_theta*1.0;
			if (w > nav_w())
				w = nav_w();
			if (w < -nav_w())
				w = -nav_w();
		}

		int ob = find_obstacle_forward(scan);
		if(ob == 1)
		{
			v=0.0;
		}
		cmd.set_v(v);
        cmd.set_w(w);
		std::cout<<"velocity"<<" ob="<<ob<<" v="<<v<<" w="<<w<<std::endl;
		this->nh->publish("set_cmd_follow",cmd);
	}

	void reset()
	{
		initial_ = true;
	}

	void update(const stdmsg::Laser_Scan & scan )
    {
		stdmsg::Pose pos;
        pos = this->cal_object_position(scan);		
		//set_goal(pos,scan);
		set_vel(pos,scan);
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
