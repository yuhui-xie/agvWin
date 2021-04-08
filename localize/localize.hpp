#ifndef LOCALIZE_HPP
#define LOCALIZE_HPP

#include <queue>
#include <stdmsg.hh>
#include "particlefilter/localizecore.h"

#include <iostream>
#include <image.h>
#include <node.hpp>
#include <thread.hpp>
#ifndef NO_ICP
#include "icp/ICP.hpp"
#include <fstream>
#endif
namespace localize
{
class processor
{
private:
#ifndef NO_ICP
    icp::ICP icp_processor;
#endif

    //for particle filter
    localize_particle_filter_p pf;
    //original grid map
    map_t raw_map;
    localize_map_t pfmap;
    localize_summary_t summary;
	localize_param_t param;
    robot_laser_message frontlaser;

	bool initialized;

	stdmsg::Laser_Scan corrected_scan;

    PARAMETER(int, particle_num);
	PARAMETER(double, mean_c_d  );   
	PARAMETER(double, mean_c_t  );  
	PARAMETER(double, std_dev_c_d); 
	PARAMETER(double, std_dev_c_t); 
	
	PARAMETER(double, mean_d_d   ); 
	PARAMETER(double, mean_d_t   ); 
	PARAMETER(double, std_dev_d_d); 
	PARAMETER(double, std_dev_d_t); 
	
	PARAMETER(double, mean_t_d   ); 
	PARAMETER(double, mean_t_t   ); 
	PARAMETER(double, std_dev_t_d); 
	PARAMETER(double, std_dev_t_t); 

	PARAMETER(double, alpha1); 
	PARAMETER(double, alpha2); 
	PARAMETER(double, alpha3); 
	PARAMETER(double, alpha4); 
	PARAMETER(double, alpha5); 

	PARAMETER(double, laser_x);
	PARAMETER(double, laser_y);
	PARAMETER(double, laser_theta);

	bool parameter_initialized;

	BMutex _lock;
	inline void lock(){this->_lock.lock();}
	inline void unlock(){this->_lock.unlock();}
private:
	void parameter();
public:

    processor();
    inline ~processor()
    {
		if(frontlaser.range)
			delete []frontlaser.range;
		if(raw_map.complete_map)
			delete[] raw_map.complete_map;
		if(raw_map.map)
			delete raw_map.map;
    }	

	//thread safe, can be call in anoter thread
	inline void initialize(const stdmsg::Pose & pose, const stdmsg::Pose & nosie)
    {
		lock();
		//this->parameter();

		////crti:原来的,2016-03-21,用到激光位置
  //      point_t mean;
		//mean.x = pose.position().x();
		//mean.y = pose.position().y();
		//mean.theta = pose.orentation().yaw();

		//double lx=-0.38, ly=0.2,lt=2.355,rt;
		//double lx=0.6, ly=-0.38,lt=-0.7854,rt;
		double lx, ly, lt, rt;
		lx = laser_x();
		ly = laser_y();
		lt = laser_theta();
		rt = pose.orentation().yaw();
		point_t mean;
		mean.x = pose.position().x() + lx * cos(rt) - ly * sin(rt);
		mean.y = pose.position().y() + lx * sin(rt) + ly * cos(rt);
		mean.theta = rt + lt;

        point_t std;
        std.x = nosie.position().x();
        std.y = nosie.position().y();;
        std.theta = nosie.orentation().yaw();

        localize_initialize_particles_gaussians(pf, 1, &mean, &std);
		unlock();
    }

	double get_laser_envir_matching_degree();

	double evaluate_result();

public:
    void set_map( Map &map );

	inline stdmsg::Laser_Scan scan()
	{
		return this->corrected_scan;
	}
	
    void update(const stdmsg::Laser_Scan& scan);

};
}

#endif
