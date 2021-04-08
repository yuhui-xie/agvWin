#include <node.hpp>
#include <stdmsg.hh>
#include <thread.hpp>
//#include "driver\LaserUST.h"
#include "driver\Laser.h"

#include <vector>
#include <iostream>
#include <sstream>
#include <math.h>
#include <queue>
#include <list>

class Laser_Test
{
protected:
	middleware::Node *nh;
private:
	struct _Node_Thread:public BThread
    {
        Laser_Test* handle;
        _Node_Thread(Laser_Test* p)
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

    struct _Update_Thread:public BThread
    {
        Laser_Test* handle;
        _Update_Thread(Laser_Test* p)
        {
            handle = p;
        }
        ~_Update_Thread()
        {
            kill();
        }
        void run()
        {
            while(true)
            {
                if(handle->laser)
                    handle->laser->update();
                //std::cerr<<"."<<std::endl;
				//Sleep(100);
                handle->seq ++;
                handle->cond.wakeup();
            }

        }
    } update_thread;

    driver::LaserLib *laser;
	BCond cond;
	unsigned int seq;

public:
	Laser_Test(int argc, char** argv):
        update_thread(this),
        nh_thread(this),
        seq(0)
    {
        laser = new driver::LaserLib("192.168.10.10", 10940);
        nh = new middleware::Node("tcp://127.0.0.1:9001");



        this->nh_thread.start();
        this->update_thread.start();
    }

    ~Laser_Test()
    {
    	if(nh)
    		delete nh;
    }

	stdmsg::Laser_Scan scan()
    {
        static stdmsg::Laser_Scan scan;
        cond.wait();
        scan.set_seq( seq );

        std::vector<double> ranges = this->laser->ranges;
        if( scan.config().angle_max() != laser->angle_max || scan.ranges_size() < ranges.size())
        {
            scan.mutable_config()->set_angle_min( laser->angle_min );
            scan.mutable_config()->set_angle_max( laser->angle_max );
            scan.mutable_config()->set_angle_increment( laser->angle_increment );
            scan.mutable_config()->set_range_max( laser->range_max );
            for(int i = scan.ranges_size(); i < ranges.size(); i ++)
            {
                scan.mutable_ranges()->Add();
            }
        }

        for(int i = 0; i < ranges.size(); i ++)
        {
            scan.mutable_ranges()->Set(i, ranges[i]);
        }
        
        scan.set_steer(0);
        scan.mutable_robot()->mutable_position()->set_x(0);
        scan.mutable_robot()->mutable_position()->set_y(0);
        scan.mutable_robot()->mutable_orentation()->set_yaw(0);
        scan.mutable_pose()->mutable_position()->set_x(0);
        scan.mutable_pose()->mutable_position()->set_y(0);
        scan.mutable_pose()->mutable_orentation()->set_yaw(0);

#ifdef DEBUG_HELPPER
        std::cout<<"ranges = [";
        for(int i = 0; i < ranges.size(); i ++)
            std::cout<<scan.ranges(i)<<",";
        std::cout<<"]"<<std::endl;
#endif
        return scan;
    }

    void update()
    {
    	while(1)
    	{
    		stdmsg::Laser_Scan scan_ = this->scan();
    		this->nh->publish("laser", scan_);
			
    		_sleep(1);
    	}
    	
    }
	int check_error(){}
	void restart_laser(){}
};

int main(int argc, char **argv)
{
	Laser_Test laser_test(argc, argv);
	laser_test.update();
	return 0;
}
