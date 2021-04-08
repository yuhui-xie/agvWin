#include "global_plan/global_plan.h"
#include "local_plan/local_plan.h"


#include <vector>
#include <iostream>

#include <image.h>
#include <node.hpp>
#include <stdmsg.hh>
#include "BThread.hpp"
inline double dist(double x, double y)
{
	return sqrt(x*x + y*y);
}

class Navigator
{
    struct Pose
    {
        Pose(double _x=0, double _y=0, double _theta=0)
            :x(_x), y(_y), theta(_theta)
        {}
        double x;
        double y;
        double theta;
    };
private:

    stdmsg::Laser_Scan laser_scan;

    BMutex _lock;// multithread read/write variable plan
    std::vector<Pose> _plans;               //multithread
    Pose _goal;
    Pose _current_pose;                     //multithread
    std::string annotation_buff;
	bool _xy_reached;
	bool _goal_reached;
    double goal_tolerance;
    double theta_tolerance;

    struct _Update_Thread:public BThread
    {
        BCond cond;
        Navigator* handle;
        _Update_Thread(Navigator* p)
        {
            handle = p;
        }
        void run()
        {
            while(true)
            {
                this->cond.wait();
                //handle->update_map();
            }

        }
        inline void begin()
        {
            this->cond.wakeup();
        }
    } update_thread;
    volatile bool is_updating;
    middleware::Node* nh;

    //local planner
    double waypoint_threshhold;
    double _history_v;
    double _history_w;
    double _max_v;
    double _max_w;

	std::string publish_topic;

	global_plan::planner gplan;
	local_plan::planner lplan;
public:
    Navigator():
        update_thread(this),
        _history_v(0),
        _history_w(0),
        _xy_reached(false),
		_goal_reached(false),
        is_updating(false)
    {

		/*
        update_thread.start();*/
		
		//global planner initialize
		Map img("map.png");
		gplan.set_map(img);

		//local planner initialize
		lplan.set_space_step(0.05);
		lplan.set_v_step(0.05);
		lplan.set_w_step(0.05);
		lplan.set_max_v(1);
		lplan.set_max_w(1);
		lplan.set_period(10);
		lplan.set_t_step(0.01);
		
		std::string host =  "tcp://*:6003";
		std::string topic = "laser";
		std::string address = "tcp://127.0.0.1:6001";
		publish_topic = "global_plan";

		nh = new middleware::Node(host);
		nh->subscrible(topic, &Navigator::update, this);
        nh->connect(address);	
    }
    ~Navigator()
    {
    }

public:
    void run()
    {
        try
        {
            while (1)
                nh->run();
        }catch(...)
        {
            std::cout<<"exit"<<std::endl;
        }
    }

    void update(const stdmsg::Laser_Scan& scan)
    {
		fprintf(stdout,"seq=%d\n",scan.seq());
		
		//global planner
        /*std::string annotation = scan.annotation();
        if(annotation.find("set_goal") == std::string::npos && annotation_buff != "")
            annotation = annotation_buff;
        if( (annotation.find("set_goal ")) != std::string::npos )
        {
            float x, y, t;
            sscanf(annotation.c_str(), "%s %f %f %f", &x, &y , &t);

            if( true )
            {
                fprintf(stderr, "set goal: robot( %.3f %.3f %.3f) ", x,y,t);
                annotation_buff = "";
            }
            else
                annotation_buff = annotation;
        }*/

		gplan.update(scan);
		const stdmsg::Global_Plan& global_path = gplan.path();
		nh->publish(publish_topic, global_path);

		//for debug
		Map path_distance("map.png");
		gplan.set_goal(1496.0/40.0, 1659.0/40.0);
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
		
		

		//local update
		lplan.set_plan( global_path );
		lplan.update(scan);
		nh->publish("cmd_vel", lplan.get_best_command() );
    }

    //thread safe
    bool replan(const stdmsg::Laser_Scan& scan)
    {
        this->_lock.lock();
        if(is_updating == true)
        {
            this->_lock.unlock();
            return false;
        }
        this->_lock.unlock();

        laser_scan = scan;
        update_thread.begin();
        return true;
    }


};



int main(int argc, char **argv)
{
	
	
	//laser.mutable_robot()->mutable_position->set_x(135.0/4.0);
	//laser.mutable_robot()->mutable_position->set_x(480/4.0);
    Navigator nav;
    nav.run();
}
