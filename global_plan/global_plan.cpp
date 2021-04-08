#include "global_plan.h"
#include "planner.h"
#include <vector>
#include <iostream>

using namespace global_plan;
using namespace std;

static map_t gmap;
static navigator_config_t gconfig;

planner::planner()
{
	gmap.complete_map = NULL;
	gmap.map = NULL;
	this->_freespace = 0;
	this->_obstacles = 1;
	this->_radius = 3;
	this->_infation = 0.6;
	this->_infation = 1;
	this->_diffsmooth = 1e-4; 
}
void planner::parameter()
{
	gconfig.map_update_freespace = this->_freespace;
	gconfig.map_update_obstacles = this->_obstacles;
	gconfig.map_update_radius = this->_radius;
	gconfig.plan_to_nearest_free_point = 0;
	gconfig.robot_width = this->_infation;
	//gconfig.robot_width = 0;
	gconfig.smooth_threshold = this->_diffsmooth;
	gconfig.smooth_path = 1;
}
planner::~planner()
{
	if(gmap.complete_map)
		delete[] gmap.complete_map;
	if(gmap.map)
		delete[] gmap.map;
	gmap.complete_map = NULL;
	gmap.map = NULL;
}
void planner::set_map(Map& img,Map& nav_img)
{
	parameter();
	gconfig.goal_size = img.resolution();

	if(gmap.complete_map)
		delete[] gmap.complete_map;
	if(gmap.map)
		delete[] gmap.map;
	gmap.complete_map = NULL;
	gmap.map = NULL;

	gmap.config.resolution  = img.resolution();
	gmap.config.x_size = img.width();
	gmap.config.y_size = img.height();
	gmap.config.map_name = NULL;
	gmap.complete_map = new float[gmap.config.x_size * gmap.config.y_size];
	gmap.map = new float* [gmap.config.x_size];
	for (int x= 0; x < img.width(); ++x)
	{
		gmap.map[x] = gmap.complete_map + x * img.height();
		for(int y = 0;y < img.height(); y++)
		{
			float *p = gmap.complete_map + x * gmap.config.y_size + y;
			unsigned int v = img(x,y);
			if(v != Map::UNKOWN )
			{   
				//if (1.0 - Map::red(v) / 255.0 > 0.3)
				if (1.0 - Map::red(v) / 255.0 > 0.6)
					*p = 1.0;
					//*p = 1.0 - Map::red(v) / 255.0;
				else
					*p = 0;
			}
			else
				*p = -1;
		}
	}
	std::cout<<"navigtion map set: "<<img.width()
		<<" X "<<img.height()<<"@"<<img.resolution()<<std::endl;

	//crti:2016-12-20,navmap
	if(gmap.complete_navmap)
		delete[] gmap.complete_navmap;
	if(gmap.navmap)
		delete[] gmap.navmap;
	gmap.complete_navmap = NULL;
	gmap.navmap = NULL;
	gmap.complete_navmap = new float[gmap.config.x_size * gmap.config.y_size];
	gmap.navmap = new float* [gmap.config.x_size];
	for (int x= 0; x < nav_img.width(); ++x)
	{
		gmap.navmap[x] = gmap.complete_navmap + x * nav_img.height();
		for(int y = 0;y < nav_img.height(); y++)
		{
			float *p = gmap.complete_navmap + x * gmap.config.y_size + y;
			unsigned int v = nav_img(x,y);
			if (v > 0 && v != Map::UNKOWN &&1.0 - Map::red(v) / 255.0 > 0.6)
			{   
				//*p = v;
				*p = 0.8;
			}
			else
				*p = -1;
		}
	}
	std::cout<<"navigtion navmap set: "<<nav_img.width()
		<<" X "<<nav_img.height()<<"@"<<nav_img.resolution()<<std::endl;

	planner_set_map(&gmap, gconfig.robot_width);

	double* ptr = conventional_get_costs_ptr();
	std::cout<<ptr<<std::endl;
	Map cost(img);
	for (int x= 0; x < cost.width(); ++x)
	{
		for(int y = 0;y < cost.height(); y++)
		{
			double *p = ptr + x*cost.height() + y;
			unsigned int v =(1.0 - *p)* 0xFF;
			cost(x,y) = 0xFF000000 + v + (v<<8) + (v<<16);
		}
	}
	cost.save("cost.png");
}
void planner::set_goal(const stdmsg::Pose& pos)
{
	parameter();
	point_t goal;
	goal.x = pos.position().x();
	goal.y = pos.position().y();
	goal.theta = pos.orentation().yaw();
	this->goalx = goal.x;
	this->goaly = goal.y;
	this->goaltheta = goal.theta;
	planner_update_goal(&goal,0,&gconfig);
}
stdmsg::Global_Plan planner::path()
{
	stdmsg::Global_Plan gplan;
	//get the plan
	planner_status_t plan;
	planner_get_status(&plan);

	for (int i = 0; i < plan.path.length; i++)
	{
		gplan.add_path();
		gplan.mutable_path(i)->mutable_position()->set_x( plan.path.points[i].x );
		gplan.mutable_path(i)->mutable_position()->set_y( plan.path.points[i].y );
		gplan.mutable_path(i)->mutable_orentation()->set_yaw( plan.path.points[i].theta );

	}

	if(gplan.path_size() > 0)
	{
		gplan.mutable_path(gplan.path_size() - 1)->mutable_position()->set_x( this->goalx );
		gplan.mutable_path(gplan.path_size() - 1)->mutable_position()->set_y( this->goaly );
	}

    if(gplan.path_size() == 0)
    {
		//Sleep(1000);
		
		//gplan.mutable_path(gplan.path_size() - 1)->mutable_position()->set_x(this->goalx);
		//gplan.mutable_path(gplan.path_size() - 1)->mutable_position()->set_y(this->goaly);
		//Sleep(1000);
		if (gplan.path_size() == 0)
		{ 
		gplan.add_path();
        gplan.mutable_path(0)->mutable_position()->set_x( 0 );
        gplan.mutable_path(0)->mutable_position()->set_y( 0 );
        gplan.mutable_path(0)->mutable_orentation()->set_yaw( 0 );

		}
    }

	if (plan.path.length > 0)
		free(plan.path.points);

	return gplan;
}
void planner::update(const stdmsg::Laser_Scan& laser_scan)
{
	parameter();
	robot_laser_message frontlaser;
	frontlaser.id = 1;

	frontlaser.config.start_angle = laser_scan.config().angle_min();
	frontlaser.config.angular_resolution = laser_scan.config().angle_increment();
	frontlaser.config.fov = laser_scan.config().angle_max() - laser_scan.config().angle_min();
	frontlaser.config.maximum_range = laser_scan.config().range_max();
	frontlaser.laser_pose.x = laser_scan.pose().position().x();
	frontlaser.laser_pose.y = laser_scan.pose().position().y();
	frontlaser.laser_pose.theta = laser_scan.pose().orentation().yaw();
	frontlaser.num_readings = laser_scan.ranges_size();

	gconfig.num_lasers_to_use = laser_scan.ranges_size();

	frontlaser.range = new float[frontlaser.num_readings];
	for(int i = 0; i < frontlaser.num_readings; i++)
	{
		frontlaser.range[i] = laser_scan.ranges(i);
		//if(frontlaser.range[i] < 0.2)
		if (frontlaser.range[i] < 0.1)
			frontlaser.range[i] = laser_scan.config().range_max();
	}

	//update obstacle use laser
	planner_update_map(&frontlaser, &gconfig);
	delete frontlaser.range;

	//robot pose have changed, update it
	traj_point_t start;
	start.x = laser_scan.robot().position().x();
	start.y = laser_scan.robot().position().y();
	start.theta = laser_scan.robot().orentation().yaw();
	planner_update_robot(&start, &gconfig );

}
void planner::reset()
{
	parameter();
	planner_reset_map(gconfig.robot_width);
}
