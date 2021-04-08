#ifndef ROUTEPLANNER_H
#define ROUTEPLANNER_H

#include <vector>
#include <iostream>
#include <sstream>
#include <image.h>
#include <node.hpp>
#include <stdmsg.hh>
#include <thread.hpp>
#include <math.h>
#include <queue>
#include <list>
#include <fstream>
#include <utility>

#include <windows.h>
#include <stdio.h>
#include "AStar.h"



class Planner
{
private:
	/*************************路径搜索相关***********************/
	double rad;
	Astar astar;
	Point start;
	Point target;
	double target_yaw;
	int is_pose_initialized;
	int is_updating_path;
public:
	std::string waypoints_file;
	stdmsg::Global_Plan planning;
	std::ofstream logfile;
	/*************************路径跟踪相关***********************/
private:
	double posx_;                                         //当前位姿
	double posy_;
	double the_;
	double nextx;                                         //下一个要跟踪的目标点
	double nexty;
	double nextbeta;
	double dist;                                          //当前位置与目标点间距离
	double delta_orien;
	double dist_th;                                       //目标点容忍度
	double orien_th;
	double len_th;                                        //前瞻
	int point_p;                                          //当前搜寻到到路径点
	int pointsize;
	int toward;

	double max_v;
	double max_w;
	double linear_coef;
	double angular_coef_1;
	double angular_coef_2;
	double angular_coef_3;
	
public:
	stdmsg::Velocity cmd;
	int goalflag;//是否达到目标
	int distflag;//是否在目标点附近

public:                                        
	Planner(middleware::ConfigFile& cfg);
	~Planner();
	void setGoal(const stdmsg::Pose &goal);
	void setPose(const stdmsg::Pose &pose);
	void initParam(middleware::ConfigFile& cfg);
	void findnextpoint();
	void calPID();
	void setToward(int val);
};

#endif
