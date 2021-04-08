#include "route_planner\route_planner.h"

int main()
{
	middleware::ConfigFile cfg("navigator.ini");
	Planner planner(cfg);
	stdmsg::Pose pose;
	pose.mutable_position()->set_x(17.4098);
	pose.mutable_position()->set_y(7.15009);
	planner.setPose(pose);

	return 0;
}