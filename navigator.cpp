#include <vector>
#include <stack>
#include <iostream>
#include <sstream>
#include <fstream>
#include <image.h>
#include <node.hpp>
#include <stdmsg.hh>
#include <thread.hpp>
#include <math.h>
#include <queue>
#include <list>

#include <windows.h>
#include <stdio.h>
#include "route_planner\route_planner.h"
#include "yaml-cpp\yaml.h"
#include <map>

using namespace std;


struct Task{
	double x;
	double y;
	double z;
	int toward;
	std::string pre;
	Task(){}
	Task(double _x, double _y, double _z, int _t, std::string _p) :x(_x), y(_y), z(_z), toward(_t), pre(_p){}

};

void operator >> (const YAML::Node& doc, Task& task){
	std::string tmp;
	doc[0] >> tmp;
	task.x = atof(tmp.c_str());
	doc[1] >> tmp;
	task.y = atof(tmp.c_str());
	doc[2] >> tmp;
	task.z = atof(tmp.c_str());
	doc[3] >> tmp;
	task.toward = atoi(tmp.c_str());
	doc[4] >> tmp;
	task.pre = tmp;
}

class TaskHandle
{
private:
	std::map<std::string, Task> content;
public:
	~TaskHandle(){};
	TaskHandle(){};
	void paser(std::string filename){
		std::ifstream fin(filename);
		YAML::Parser parser(fin);
		YAML::Node doc;
		parser.GetNextDocument(doc);
		for (YAML::Iterator it = doc.begin(); it != doc.end(); ++it) {
			std::string key;
			Task task;
			it.first() >> key;
			it.second() >> task;
			content[key] = task;
		}
	}

	void set(std::string key, Task task)
	{
		this->content[key] = task;
	}

	Task& operator[](std::string key){
		return this->content[key];
	}

	void save(std::string filename){
		YAML::Emitter yaml_out;
		std::ofstream fout(filename, std::ios::out);
		yaml_out << YAML::BeginMap;
		for (std::map<std::string, Task>::iterator iter = this->content.begin(); iter != this->content.end(); iter++){
			std::cout << "saving:" << iter->first << std::endl;
			yaml_out << YAML::Key << iter->first;
			yaml_out << YAML::Value;
			auto task = iter->second;
			yaml_out << YAML::BeginSeq << task.x << task.y << task.z << task.toward << task.pre << YAML::EndSeq;
		}
		yaml_out << YAML::EndMap;
		fout << yaml_out.c_str();
		this->content.clear();
	}

	bool has_value(std::string key){
		auto iter = this->content.find(key);
		return iter != this->content.end();
	}
};


// 运动多个目标点后，到达最终目标点，保存数据用的
// 其实没有太大必要，有时间再简化掉8
class MultiStageGoalSvaer
{
private:
	std::stack<stdmsg::Pose> pose_stack;
	std::stack<int> toward_stack;

public:
	void push(stdmsg::Pose pose, int toward){
		this->pose_stack.push(pose);
		this->toward_stack.push(toward);
	}

	void pop(stdmsg::Pose *pose, int *toward){
		*pose = pose_stack.top();
		*toward = toward_stack.top();
		pose_stack.pop();
		toward_stack.pop();
	}

	int empty(){
		return this->pose_stack.empty();
	}

	void clear(){
		this->pose_stack.swap(std::stack<stdmsg::Pose>());
		this->toward_stack.swap(std::stack<int>()); 
	}
};

class App
{
private:
	middleware::Node *nh;  // 通信节点
	middleware::ConfigFile cfg;  // 配置文件
    struct _Node_Thread: public BThread
    {
        App* handle;
        _Node_Thread(App* p)
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
    } nh_thread;  // 跑通信节点的线程
	Planner *planner;
    
	std::string correct_scan_topic; 
	std::string goal_topic;

	std::string path_file;  // 保存了路径的文件名
	std::string goal_file;
	std::string task_file;

	MultiStageGoalSvaer goal;  // 存储一系列目标点
	int is_record;  // 模式
	std::string cur_place;  // 当前地点
	stdmsg::Pose cur_pos;  // 当前位姿

	int goal_setting_flag;

public:
	App(int argc, char** argv)
		:nh_thread(this)
    {
		std::string cfgfile("navigator.ini");
		for (int i = 0; i < argc - 1; i++)
			if( strcmp(argv[i],"-cfg") == 0 )
				cfgfile = argv[i+1];
		cfg.read(cfgfile);
		planner = new Planner(cfg);

		//node set and connect, copy from robot.cpp
		std::string tmp = cfg.value("node", "bind_nav", "tcp://127.0.0.1:9002");
		std::cout<<"[node] bind = "<< tmp <<std::endl;
		nh = new middleware::Node(tmp);
		
		tmp = std::string(cfg.value("node", "connect_nav", "tcp://127.0.0.1:9001"));
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
        correct_scan_topic  = std::string( cfg.value("topic", "scan", "scan") );
		goal_topic = std::string(cfg.value("topic", "goal", "goal"));    ///////  new

        nh->subscrible(correct_scan_topic, &App::planner_pose_callback, this);
		nh->subscrible(goal_topic, &App::set_goal_callback, this);
		path_file = std::string(cfg.value("route_planner", "waypoints_file", "path.txt"));
		goal_file = std::string(cfg.value("route_planner", "goal_file", "goal_places.yaml"));
		task_file = std::string(cfg.value("route_planner", "task_file", "task.txt"));

		cur_place = middleware::ConfigFile(task_file).value("initial_place", "place", "bed");
		is_record = 1;
		goal_setting_flag = 0;
		this->nh_thread.start();
    }
	~App()
	{
		std::cerr << "node uninitialize!" << std::endl;
		if (nh)
			delete nh;

		std::cerr << "nav uninitialize!" << std::endl;
		if (planner)
			delete planner;
	}
	
	// 位姿回调函数
	void planner_pose_callback(const stdmsg::Laser_Scan &laser){
		double x = laser.robot().position().x();
		double y = laser.robot().position().y();
		double theta = laser.robot().orentation().yaw();

		// 打印信息，调试用
		static int robot_pos_cnt = 0;
		if (robot_pos_cnt < 10){ robot_pos_cnt++;  }
		else{
			std::cout << "robot_pose(x, y, theta): " << "( " << x << ", " << y << ", " << theta << " )" << std::endl;
			std::cout << "robot_vel_cmd(v, w): " << "( " << this->planner->cmd.v() << ", " << this->planner->cmd.w() << " )" << std::endl;
			robot_pos_cnt = 0;
		}

		this->cur_pos.mutable_position()->set_x(x);
		this->cur_pos.mutable_position()->set_y(y);
		this->cur_pos.mutable_orentation()->set_yaw(theta);
		this->planner->setPose(this->cur_pos);
		stdmsg::Velocity cmd = this->planner->cmd;
		this->nh->publish("set_cmd", cmd);
		/*
		// 模式1：记录路径
		if (this->is_record == 1){
			stdmsg::Velocity cmd;
			cmd.set_v(0.0);
			cmd.set_w(0.0);
			this->nh->publish("set_cmd", cmd);
		}
		// 模式0：发布速度
		else if (this->is_record == 0){
			stdmsg::Velocity cmd = this->planner->cmd;
			this->nh->publish("set_cmd", cmd);
		}
		*/
	}

	// 模式回调函数
	void mode_callback(const stdmsg::MoveMode mode){
		this->is_record = mode.move_mode();
	}

	// 从goal.txt中读取出目标位置，然后再到前面的配置文件里找对应的坐标，调试用。
	void set_goal_from_file()
	{
		middleware::ConfigFile task_file("task.txt");
		stdmsg::Pose goal_pose;
		std::string place = std::string(task_file.value("goal", "place", "toilet"));
		int toward;
		if (place == cur_place){ 
			return; 
		}
		else{ 
			cur_place = place; 
			goal.clear();
			std::cout << "recv place: " << place << std::endl;
		}

		while (place != "None"){
			goal_pose.mutable_position()->set_x(double(task_file.value(place, "x", "0")));
			goal_pose.mutable_position()->set_y(double(task_file.value(place, "y", "0")));
			goal_pose.mutable_orentation()->set_yaw(double(task_file.value(place, "z", "0")));
			toward = (int)task_file.value(place, "toward", 1);
			goal.push(goal_pose, toward);
			std::cout << "successfully push: " << place << std::endl;
			place = task_file.value(place, "pre", "None");
		}
		
		while (!goal.empty()){
			goal.pop(&goal_pose, &toward);
			std::cout << "the target place is: " << cur_place <<  ", with pose:" << goal_pose.position().x() << ' ' << goal_pose.position().y() << ' ' << goal_pose.orentation().yaw() << std::endl;
			std::cout << "with toward: " << toward << std::endl;
			this->planner->setToward(toward);
			this->planner->setGoal(goal_pose);
			while (!this->planner->goalflag){
				std::cout << "on the way..." << std::endl;
			}
		}
	} 
	
	// goal回调
	void set_goal_callback(const stdmsg::String &goal_place)
	{
		this->goal_setting_flag = 1;
		std::string place = goal_place.str();
		std::cout << "now navigator has recv place: " << place << std::endl;
		middleware::ConfigFile task_file(this->task_file);
		TaskHandle th;
		th.paser(this->goal_file);

		this->goal.clear();
		int toward;
		stdmsg::Pose goal_pose;
		while (th.has_value(place) && place != "None"){
			Task goal_task = th[place];
			goal_pose.mutable_position()->set_x(goal_task.x);
			goal_pose.mutable_position()->set_y(goal_task.y);
			goal_pose.mutable_orentation()->set_yaw(goal_task.z);
			toward = goal_task.toward;
			goal.push(goal_pose, toward);
			std::cout << "successfully push: " << place << std::endl;
			std::cout << "target is: " << goal_task.x << ' ' << goal_task.y << ' ' << goal_task.z << std::endl;
			place = goal_task.pre;
		}
		this->goal_setting_flag = 0;
	}

	// 用planner计算路径
	void send_goal_to_planner(){
		if (this->goal_setting_flag){
			return;
		}

		if (!goal.empty() && this->planner->goalflag){
			stdmsg::Pose goal_pose;
			int toward;
			goal.pop(&goal_pose, &toward);
			std::cout << "running..." << std::endl;
			std::cout << "the target place is: " << cur_place << ", with pose:" << goal_pose.position().x() << ' ' << goal_pose.position().y() << ' ' << goal_pose.orentation().yaw() << std::endl;
			std::cout << "with toward: " << toward << std::endl;
			this->planner->setToward(toward);
			this->planner->setGoal(goal_pose);
		}
	}

	// main
	void spin(){
		while (1){
			send_goal_to_planner();
		}
	}
};


int main(int argc, char **argv)
{
	App nav(argc, argv);
	nav.spin();
}
