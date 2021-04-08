#include "localize/localize.hpp"
#include "robot.h"
#include "slam/slam.hpp"
#include <vector>
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
#include <memory>
#include <windows.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include "yaml-cpp\yaml.h"

std::string place_mapping[7] = {" ", "toilet", "bed", "desk", "sofa", "fridge", "balcony"};

struct Task{
	double x;  // 目标的位姿
	double y;
	double z;
	int toward;  // =1正向移动到目标，=0反向运动到目标
	std::string pre;  // 运动到目标前应先运动到哪个位姿上
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
	// 读入当前yaml文件中所有存储值
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
	// 修改某个变量
	void set(std::string key, Task task)
	{
		this->content[key] = task;
	}

	Task& operator[](std::string key){
		return this->content[key];
	}

	bool has_value(std::string key){
		auto iter = this->content.find(key);
		return iter != this->content.end();
	}

	// 将当前存储值以yaml的格式写到目标文件中
	void save(std::string filename){
		YAML::Emitter yaml_out;
		std::ofstream fout(filename, std::ios::out);
		yaml_out << YAML::BeginMap;
		for (std::map<std::string, Task>::iterator iter = this->content.begin(); iter != this->content.end(); iter++){
			yaml_out << YAML::Key << iter->first;
			yaml_out << YAML::Value;
			auto task = iter->second;
			yaml_out << YAML::BeginSeq << task.x << task.y << task.z << task.toward << task.pre << YAML::EndSeq;
		}
		yaml_out << YAML::EndMap;
		fout << yaml_out.c_str();
		this->content.clear();
	}
};

/*
Localize类继承自processor类
override构造函数：
1. 定义了各个参数的值
2. 设置了地图
*/
class Localize : public localize::processor
{
private:
	middleware::Node* nh;
public:
	Localize(middleware::ConfigFile& cfg) : localize::processor()
	{
		//localize initialize
#undef helper__
#define helper__(type, x, _default) this->set_##x ( type(cfg.value("motion_model", #x, _default) ) );\
	std::cout << "[motion_model] "#x << "\t = (" << #type << ") " << this->x() << std::endl;
		helper__(double, mean_c_d, -0.0123);
		helper__(double, mean_c_t, -0.1065);
		helper__(double, std_dev_c_d, 0.1380);
		helper__(double, std_dev_c_t, 0.2347);

		helper__(double, mean_d_d, 1.0055);
		helper__(double, mean_d_t, 0.0025);
		helper__(double, std_dev_d_d, 0.1925);
		helper__(double, std_dev_d_t, 0.3982);

		helper__(double, mean_t_d, -0.0025);
		helper__(double, mean_t_t, 0.9638);
		helper__(double, std_dev_t_d, 0.0110);
		helper__(double, std_dev_t_t, 0.3300);

		helper__(double, alpha1, 6.0);
		helper__(double, alpha2, 6.0);
		helper__(double, alpha3, 18.0);
		helper__(double, alpha4, 6.0);
		helper__(double, alpha5, 3.0);

#undef helper__
#define helper__(type, x, _default) this->set_##x ( type(cfg.value("laser_pose", #x, _default) ) );\
	std::cout << "[laser_pose] "#x << "\t = (" << #type << ") " << this->x() << std::endl;
		//crti:2016-05-06,激光相对几何中心位置
		helper__(double, laser_x, 0.6);
		helper__(double, laser_y, -0.38);
		helper__(double, laser_theta, -0.7854);

		//setup maps
		std::string mapname = cfg.value("map", "localize", "map.png");
		std::cout << "[map] localize\t = (string) " << mapname << std::endl;
		Map localizemap(mapname);
		this->set_map(localizemap);
	}
};


class LocalizableRobot : public Robot
{
	Localize *localization;
	std::string path_file;  // 保存了路径的文件名
	std::string task_file;  // 保存了任务的文件名，调试用
	std::string goal_file;  // 保存了目标的文件名
	stdmsg::Pose back_pose;  // 用于记录开始反向运动的点
	int back_flag;

	int is_mapping;  // 是否在建图
	SLAM::Slam *slam;

	std::string correct_scan_topic;  // 发布话题
	std::string mode_topic;
	std::string goal_topic;

	std::string cur_place;  /// 当前位置

	std::string cur_setted_place;  // 要设置的目标位置
	int setted_place_index;

	int is_record;  // 0：录制路径, 1：规划路径
	
	stdmsg::Pose pose;

	stdmsg::Velocity cur_vel;
	int acc_N;  // 经过N步后加速到目标速度
	int cnt_N;  // 当前加速步数计数器
	double acc_dv;  // 每次加速的步长

	double obstacle_forward_angle, obstacle_forward_min_distance, obstacle_forward_max_distance, obstacle_forward_points;  // 停障相关
	int obstacle_stop;

	struct _CMD_Thread : public BThread
	{
		LocalizableRobot* handle;
		_CMD_Thread(LocalizableRobot* p)
		{
			handle = p;
		}
		~_CMD_Thread()
		{
			kill();
		}
		void run()
		{
			try{
				while (true)
				{
					if (!handle->obstacle_stop)
					{
						handle->accelerate_to_cmd();
						_sleep(10);
					}
				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "cmd_ error: " << e.what() << std::endl;
			}
		}
	} cmd_thread;  // 跑加速的线程

public:
	LocalizableRobot(int argc, char** argv) :
		localization(NULL),
		back_flag(0),
		cmd_thread(this),
		Robot(argc, argv)
	{
		localization = new Localize(cfg);

		correct_scan_topic = std::string(cfg.value("topic", "scan", "scan"));
		mode_topic = std::string(cfg.value("topic", "mode", "mode"));
		goal_topic = std::string(cfg.value("topic", "goal", "goal"));

		path_file = std::string(cfg.value("route_planner", "waypoints_file", "path.txt"));
		task_file = std::string(cfg.value("route_planner", "task_file", "task.txt"));
		goal_file = std::string(cfg.value("route_planner", "goal_file", "goal_places.yaml"));

		is_mapping = 0;
		is_record = 0;

		acc_N = (int)cfg.value("route_planner", "acc_N", 10);
		cnt_N = 1;
		nh->subscrible("set_cmd", &LocalizableRobot::set_cmd, this);

		set_initial_pose_from_file();
		//set_initial_pose();
		stdmsg::String move_mode;
		move_mode.set_str("AUTONOMOS");
		
		cur_vel.set_v(0);
		cur_vel.set_w(0);
		
		obstacle_forward_angle = double(cfg.value("obstacle_forward", "angle", 60.0));
		obstacle_forward_min_distance = double(cfg.value("obstacle_forward", "min_distance", 0.06));
		obstacle_forward_max_distance = double(cfg.value("obstacle_forward", "max_distance", 0.25));
		obstacle_forward_points = int(cfg.value("obstacle_forward", "points", 50));

		obstacle_stop = 0;

		this->set_mode(move_mode);
		this->start_ipc();
		this->cmd_thread.start();
		
	}

	~LocalizableRobot()
	{
		std::cerr << "localizer's node uninitialize!" << std::endl;
		if (nh)
			delete nh;

		std::cerr << "localization uninitialize!" << std::endl;
		if (localization)
			delete localization;
	}

	// 计算加速度
	void set_cmd(const stdmsg::Velocity &cmd)
	{
		this->cur_vel.set_w(cmd.w());
		this->acc_dv = (cmd.v() - this->cur_vel.v()) / this->acc_N;
		this->cnt_N = 0;
		// this->set_cmd_autonomous(cmd);
	}

	// 交给线程干，不断加速，直到达到目标速度
	void accelerate_to_cmd(){
		if (cnt_N < acc_N){
			this->cur_vel.set_v(this->acc_dv + this->cur_vel.v());
			cnt_N += 1;
		}
		this->set_cmd_autonomous(this->cur_vel);
	}

	// 在前方找到障碍物
	int find_obstacle_forward(const stdmsg::Laser_Scan & rscan)
	{
		int cnt = 0;
		double angle = obstacle_forward_angle;//角度范围，60.0表示-30度到+30度
		double min_distance = obstacle_forward_min_distance;
		double max_distance = obstacle_forward_max_distance;//距离范围，单位米
		int min_laser, max_laser;
		//crti:2016-08-16,原来是按照1080线算的，现在变成动态的了
		min_laser = (int)((rscan.ranges().size() - 1) / 2 + 1 - angle * 0.5 * 4);
		if (min_laser < 0){ min_laser = 0; std::cout << "ob,out range,<" << std::endl; }
		max_laser = (int)((rscan.ranges().size() - 1) / 2 + 1 + angle * 0.5 * 4);
		if (max_laser > rscan.ranges().size()){ max_laser = rscan.ranges().size(); std::cout << "ob,out range,>" << std::endl; }
		//std::cout << "rscan.ranges().size(): " << rscan.ranges.size() << std::endl;
		
		for (int i = min_laser; i < max_laser; i++)
		{
			double r = rscan.ranges(i);
			if (r > min_distance && r < max_distance)
			{
				++cnt;
			}
		}

		if (cnt>obstacle_forward_points) // 60度一共240个点，50个点在范围内就认为有物体
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}

	// 从文件中读入设置初始位姿，调试用
	void set_initial_pose_from_file()
	{
		stdmsg::Pose pose;
		middleware::ConfigFile data_file(task_file);
		std::string place = std::string(data_file.value("initial_place", "place", "initial_place"));
		this->cur_place = place;

		TaskHandle th;
		std::cout << "wow here im" << std::endl;
		th.paser(goal_file);
		pose.mutable_position()->set_x(th[place].x);
		pose.mutable_position()->set_y(th[place].y);
		pose.mutable_orentation()->set_yaw(th[place].z);
		stdmsg::Pose n;
		if (this->localization)
			this->localization->initialize(pose, n);
	}

	// 从面板上设置初始位姿
	void set_initial_pose(){
		int initial_pose_index = 0;
		while (initial_pose_index == 0){ 
			std::cout << initial_pose_index << std::endl;
			std::cout << "waiting for intialization!!!" << std::endl; 
			//std::cout << this->get_initial_pose();
			initial_pose_index = this->get_initial_pose();
		}
		stdmsg::Pose pose;
		std::string place = place_mapping[initial_pose_index];
		std::cout << "now the robot is in the place: " << place << std:: endl;
		this->cur_place = place;
		TaskHandle th;
		th.paser(goal_file);
		pose.mutable_position()->set_x(th[place].x);
		pose.mutable_position()->set_y(th[place].y);
		pose.mutable_orentation()->set_yaw(th[place].z);
		stdmsg::Pose n;
		if (this->localization)
			this->localization->initialize(pose, n);
	}

	// 从文件中读入设置目标的消息，调试用
	void read_goal_record_from_file(){
		middleware::ConfigFile data_file(task_file);
		this->setted_place_index = int(data_file.value("set_goal", "set", 0));
		this->cur_setted_place = data_file.value("set_goal", "name", "toilet");
	}

	// 从文件中读入模式，调试用
	void set_mode_from_file(){
		middleware::ConfigFile data_file(task_file);
		this->is_record = int(data_file.value("mode", "is_record", 1));
	}

	// 记录路径点
	void record_path(stdmsg::Pose pose){
		double x = pose.position().x();
		double y = pose.position().y();
		
		static double last_x = 0, last_y = 0;
		if (sqrt((last_x - x) * (last_x - x) + (last_y - y) * (last_y - y)) < 0.05){
			return;
		}
		last_x = x;
		last_y = y;
		
		std::ofstream data_file;
		data_file.open(path_file, std::ios::out | std::ios::app);
		data_file << x << ' ' << y << std::endl;
		// 关闭文档
		data_file.close();
		std::cout << "successfully write path to the file~" << std::endl;
		if (this->odomvelocity().v() < -0.01 && this->back_flag == 0){
			this->back_flag = 1;
			this->back_pose = pose;
		}
	}

	// 保存初始坐标点
	stdmsg::Pose read_initial_pose(){
		ifstream fin;
		fin.open("landmarks.txt");
		double x, y, z;
		fin >> x >> y >> z;
		TaskHandle th;
		th.paser(this->goal_file);
		Task task;
		task.x = x;
		task.y = y;
		task.z = z;
		task.pre = "None";
		task.toward = 1;
		th["initial_pose"] = task;
		th.save(this->goal_file);

		stdmsg::Pose initial_pose;
		initial_pose.mutable_position()->set_x(x);
		initial_pose.mutable_position()->set_y(y);
		initial_pose.mutable_orentation()->set_yaw(z);
		return initial_pose;
	}

	void stop(){
		this->cur_vel.set_v(0);
		this->cur_vel.set_w(0);
		this->set_cmd_autonomous(cur_vel);
	}

	// main
	void run_process()
	{
		while (1){
			stdmsg::Laser_Scan scan = this->scan();
			
			// 检查是否建图
			int mapping_cmd = this->get_mapping_cmd();
			//std::cout << "get cmd success" << std::endl;
			// 结束建图
			if (this->is_mapping && !mapping_cmd)
			{
				std::cout << "end mapping" << std::endl;
				this->slam->maker("end");
				this->slam->generate_map("map.png", "mapnav.png");
				this->is_mapping = 0;
				
				free(slam);
				stdmsg::Pose initial_pose = read_initial_pose();
				stdmsg::Pose n;
				if (this->localization)
					this->localization->initialize(pose, n);
				continue;
			}
			// 开始建图
			else if(!this->is_mapping && mapping_cmd){
				std::cout << "start mapping" << std::endl;
				this->slam = new SLAM::Slam(this->cfg);
				_sleep(3000);
				this->slam->maker("initial");
				this->is_mapping = 1;
				continue;
			}
			// 建图中
			else if (this->is_mapping && mapping_cmd){
				std::cout << "mapping" << std::endl;
				this->slam->update(scan);
				_sleep(1);
				continue;
			}

			// 发布模式
			// set_mode_from_file();
			this->is_record = this->get_record_path_cmd();
			stdmsg::MoveMode robot_mode;
			robot_mode.set_move_mode(this->is_record);
			this->nh->publish(mode_topic, robot_mode);

			// 进行定位
			this->localization->update(scan);
			stdmsg::Laser_Scan correct_scan = this->localization->scan();
			std::cout << "matching degree: " << this->localization->get_laser_envir_matching_degree() << std::endl;

			// 记录路径模式
			if (this->is_record == 1){
				this->record_path(correct_scan.pose());
				// 检查记录终点
				// read_goal_record_from_file();
				this->setted_place_index = this->get_set_place_cmd();
				if (this->setted_place_index > 0){
					this->cur_setted_place = place_mapping[this->setted_place_index];
					std::cout << "saving place" << std::endl;
					TaskHandle th;
					th.paser(goal_file);
					Task final_goal(
						correct_scan.pose().position().x(),
						correct_scan.pose().position().y(),
						correct_scan.pose().orentation().yaw(),
						1,
						"None"
						);
					// 如果发现有反向运动，则要将转向点设置为临时目标，修改向最终目标运动时的朝向
					if (this->back_flag){
						final_goal.pre = "pre" + this->cur_setted_place;
						final_goal.toward = 0;
						Task tmp_goal(
							this->back_pose.position().x(),
							this->back_pose.position().y(),
							this->back_pose.orentation().yaw(),
							1,
							"None"
							);
						th[this->cur_setted_place] = final_goal;
						th["pre" + this->cur_setted_place] = tmp_goal;
					}
					else{
						th[this->cur_setted_place] = final_goal;
					}
					th.save(goal_file);
				}
			}	

			else{
				this->back_flag = 0;

				// 检查停障
				if (this->find_obstacle_forward(correct_scan)){
					obstacle_stop = 1;
					stop();
				}
				else{
					obstacle_stop = 0;
				}

				// 发布位姿
				this->nh->publish(correct_scan_topic, correct_scan);
				// 发布目标
				int goal_index = this->get_goal_port();
				// std::cout << " this is goal_index : " << goal_index << std::endl;
				if (place_mapping[goal_index] != this->cur_place){
					stdmsg::String goal_place;
					std::cout << "current_place is: " << this->cur_place << std::endl;
					std::cout << "goal is: " << place_mapping[goal_index] << std::endl;
					goal_place.set_str(place_mapping[goal_index]);
					this->nh->publish(goal_topic, goal_place);
					this->cur_place = place_mapping[goal_index];
				}
			}

			
		}
	}
};

int main(int argc, char **argv){
	LocalizableRobot lr(argc, argv);
	lr.run_process();
}
