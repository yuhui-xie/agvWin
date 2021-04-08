#include "../local_plan/local_plan.h"
#include "../global_plan/global_plan.h"

#include <vector>
#include <iostream>

#include <image.h>
#include <node.hpp>
#include <stdmsg.hh>


inline double dist(double x, double y)
{
	return sqrt(x*x + y*y);
}

int main(int argc, char **argv)
{
	middleware::RPC rpc;
	std::string tmp = "tcp://127.0.0.1:9000";

	rpc.connect(tmp);
	//stdmsg::Pose pos = rpc.call<stdmsg::Pose, stdmsg::Pose> ("initial_pose", stdmsg::Pose());
	stdmsg::Pose pos;
	int x = 12;
	
	
	//while(true)
	{
		pos.mutable_position()->set_x(32.695);
		pos.mutable_position()->set_y(12.569);
		pos.mutable_orentation()->set_yaw(425.112);
		stdmsg::Pose r;
		try{
			 r = rpc.call<stdmsg::Pose, stdmsg::Pose> ("initial_pose", pos);
		}catch(std::exception &e)
		{
			std::cout<<e.what()<<std::endl;
			//exit(-1);
		}
		std::cout<<r.orentation().yaw()<<std::endl;
	}

	global_plan::planner gplan;

	Map img("map.png");
	gplan.set_map(img);

	Map path_distance("map.png");
	//gplan.set_goal(1496.0/40.0, 1659.0/40.0);
	//gplan.update(scan);
	const stdmsg::Global_Plan& global_path = gplan.path();

	for ( int i = 1; i < global_path.path_size(); ++i)
	{
		double startx = global_path.path(i - 1).position().x();
		double starty = global_path.path(i - 1).position().y();
		double endx = global_path.path(i).position().x();
		double endy = global_path.path(i).position().y();
		double length = dist( endx - startx, endy - starty);
		for (double walk = 0;  walk < length; walk += path_distance.resolution()/2.0 )
		{
			double x = startx + walk / length * (endx - startx);
			double y = starty + walk / length * (endy - starty);
			int v =(1.0 - 1)* 0xFF;
			path_distance( x / path_distance.resolution(), y / path_distance.resolution() ) = 0xFF000000 + (v<<16) + (v<<8) + v ;
		}
	}
	path_distance.save("globalplan.png");

	local_plan::planner lplan;
	
	//local planner initialize
	lplan.set_space_step(0.05);
	lplan.set_v_step(0.05);
	lplan.set_w_step(0.05);
	lplan.set_max_v(1);
	lplan.set_max_w(1);
	lplan.set_period(10);
	lplan.set_t_step(0.01);
	std::vector<local_plan::point> f;
	f.push_back( local_plan::point( 0, -1) );
	f.push_back( local_plan::point( 0, 1) );
	f.push_back( local_plan::point( 1, 1) );
	f.push_back( local_plan::point( 2, 0) );
	f.push_back( local_plan::point( 1, -1) );
	lplan.set_footprint( f );

	lplan.get_best_command();

	//local update
	//lplan.set_plan( global_path );
	//lplan.update(scan);
	//nh->publish("cmd_vel",  );
}
