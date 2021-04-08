#ifndef GLOBAL_PLAN_H
#define GLOBAL_PLAN_H
#include <vector>
#include <image.h>
#include <stdmsg.hh>
#include <node.hpp>
//this planner not spport multithread
namespace global_plan
{
class planner
{
public:
    planner();
    ~planner();
    void set_map(Map& img,Map& nav_img);
	void set_goal(const stdmsg::Pose& pos);
	void update(const stdmsg::Laser_Scan& laser_scan);
	stdmsg::Global_Plan path();
	void reset();
private:
	double goalx;
	double goaly;
	double goaltheta;
	
	// whether to clear the map by laser
	PARAMETER(bool, freespace);
	//whether to register obstacle by laser
	PARAMETER(bool, obstacles);
	//the region update 
	PARAMETER(double, radius);
	//how much the distance from the obstacle is not usable
	PARAMETER(double, infation);
	//try to smooth the path, if the paths' difference is below this, typicaly 1e-6
	PARAMETER(double, diffsmooth);
	void parameter();
};

}
#endif