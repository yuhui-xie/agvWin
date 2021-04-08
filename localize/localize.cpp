#include "localize.hpp"
using namespace localize;
processor::processor()
		:initialized(false),parameter_initialized(false)
{
	frontlaser.range = NULL;
	raw_map.complete_map = NULL;
	raw_map.map = NULL;

    param.rear_laser_offset        = 0;
    param.use_rear_laser           = 0;

    param.num_particles            =  2024;//例子数越大越好 但要考虑计算效率//crti
    param.min_wall_prob            = 0.25;

    param.outlier_fraction         = 0.9;
    param.update_distance          = 0.08;
    param.update_angle             = 0.08;
        
    param.do_scanmatching          = 0;
    param.constrain_to_map         = 1;
    param.occupied_prob            = 0.5;
    param.lmap_std                 = 0.3;
    param.global_lmap_std          = 0.3;
    param.global_evidence_weight   = 0.01;
    param.global_distance_threshold    = 1.8;
    param.global_test_samples          = 100000;
    param.use_sensor                   = 1;
	param.tracking_beam_minlikelihood = 0.45;//0.45;
    param.global_beam_minlikelihood    = 0.9;
    param.motion_model = localize_motion_initialize(0, NULL);

	summary.mean.x = 0;
	summary.mean.y = 0;
	summary.mean.theta = 0;

    param.motion_model = (localize_motion_model_t *)calloc(1, sizeof(localize_motion_model_t));
		
    pf = localize_particle_filter_new(&param);
	param.laser_skip = 1;
}
void processor::parameter()
{
	if(parameter_initialized)
		return;
	parameter_initialized = true;
	#define helper__(x) param.motion_model-> x = _##x
	helper__(mean_c_d);
    helper__(mean_c_t )    ;
    helper__(std_dev_c_d)  ;
    helper__(std_dev_c_t)  ;

    helper__(mean_d_d)     ;
    helper__(mean_d_t)     ;
    helper__(std_dev_d_d)  ;
    helper__(std_dev_d_t)  ;

    helper__(mean_t_d)     ;
    helper__(mean_t_t)     ;
    helper__(std_dev_t_d)  ;
    helper__(std_dev_t_t)  ;

	helper__(alpha1);
	helper__(alpha2);
	helper__(alpha3);
	helper__(alpha4);
	helper__(alpha5);
	#undef helper__
}
void processor::update(const stdmsg::Laser_Scan& scan)
{
	lock();
	if(!raw_map.complete_map)
		return;
	if(initialized == false)
	{
		initialized = true;
		this->parameter();
		param.max_range                = scan.config().range_max();
		param.integrate_angle          = scan.config().angle_increment();
		double dx = scan.pose().position().x() - scan.robot().position().x();
		double dy = scan.pose().position().y() - scan.robot().position().y();
		double dtheta = atan2(dy, dx) - scan.robot().orentation().yaw();
		double offset = sqrt(dx * dx + dy * dy);
		while(dtheta >  M_PI)dtheta -= M_PI * 2;
		while(dtheta < -M_PI)dtheta += M_PI * 2;
		//double dhead = scan.robot().orentation().yaw() - scan.pose().orentation().yaw();
		if(fabs(dtheta) >0.05 && fabs(fabs(dtheta) - 3.1415926) > 0.05)
		{
			std::cerr<< "the laser should be placed on the x axis"<<std::endl;
            std::cerr<< "angel is " <<dtheta<<"; "<<"offset is "<<offset<<std::endl;
            //exit(-1);
		}
		////crti:原来的,2016-03-23
		//param.front_laser_offset       =  fabs(dtheta) < 0.05 ? offset : -offset;
		param.front_laser_offset=0.0;
		frontlaser.range = new float[scan.ranges_size() + 128];
	}


    frontlaser.id = 1;
    frontlaser.config.start_angle = scan.config().angle_min();
    frontlaser.config.angular_resolution = scan.config().angle_increment();
    frontlaser.config.fov = scan.config().angle_max() - scan.config().angle_min();
    frontlaser.config.maximum_range = scan.config().range_max();

    frontlaser.laser_pose.x = scan.pose().position().x();
    frontlaser.laser_pose.y = scan.pose().position().y();
    frontlaser.laser_pose.theta = scan.pose().orentation().yaw();

    frontlaser.robot_pose.x = scan.robot().position().x(); // change to the robot coordinate
    frontlaser.robot_pose.y = scan.robot().position().y(); //change to the robot coordinate
    frontlaser.robot_pose.theta = scan.robot().orentation().yaw();//change to the robot coordinate

    frontlaser.num_readings = scan.ranges_size();//abs(int(frontlaser.config.fov / frontlaser.config.angular_resolution) + 1);
        
    for(int i = 0; i < frontlaser.num_readings ; ++i)
        frontlaser.range[i] = scan.ranges(i);

	////crti:原来的,2016-03-21,这里不能改backward
    localize_run(pf, &pfmap, &frontlaser, pf->param->front_laser_offset, 0);
	//localize_run(pf, &pfmap, &frontlaser, pf->param->front_laser_offset, 1);
	
    if(pf->initialized){
        localize_summarize(pf, &summary, &pfmap, frontlaser.num_readings,
                            frontlaser.range, pf->param->front_laser_offset,
                            frontlaser.config.angular_resolution,
                            frontlaser.config.start_angle, 0);
    }

#define NO_ICP
#ifndef NO_ICP
    std::vector<icp::ICP_Point> points;f
    for( int i = 0; i < scan.ranges_size(); i++)
    {
        double r = scan.ranges(i);
        double t = i * scan.config().angle_increment() + scan.config().angle_min() + summary.mean.theta;
        double x = r * cos(t) + summary.mean.x;
        double y = r * sin(t) + summary.mean.y;
        points.push_back( icp::ICP_Point(x, y) );
    }
    icp_processor.set_outlier(0.2);
    icp_processor.initial_pose(0,0,0);
    icp::ICP_Pose err_pose = icp_processor.match(points);
    if(fabs(err_pose.x) < 0.3 &&
            fabs(err_pose.y) < 0.3 &&
            fabs(err_pose.theta) < 0.5)
    {
        for(int pf_i = 0; pf_i < pf->param->num_particles; pf_i++)
        {
            pf->particles[pf_i].x += err_pose.x;
            pf->particles[pf_i].y += err_pose.y;
            pf->particles[pf_i].theta += err_pose.theta;
        }
    }
#endif
	this->corrected_scan = scan;
	
	////crti:原来的,2016-03-21,用到激光位置
	////pose is the laser's pose, which is the summary
	//this->corrected_scan.mutable_robot()->mutable_position()->set_x(summary.mean.x);
	//this->corrected_scan.mutable_robot()->mutable_position()->set_y(summary.mean.y);
	//this->corrected_scan.mutable_robot()->mutable_orentation()->set_yaw(summary.mean.theta);
	////robot's pose is the pose of robot base, which may not equal to the laser's pose
	//this->corrected_scan.mutable_pose()->mutable_position()->set_x(
	//	summary.mean.x + param.front_laser_offset * cos(summary.mean.theta) );
	//this->corrected_scan.mutable_pose()->mutable_position()->set_y(
	//	summary.mean.y + param.front_laser_offset * sin(summary.mean.theta) );
	//this->corrected_scan.mutable_pose()->mutable_orentation()->set_yaw(summary.mean.theta );

	//pose is the laser's pose, which is the summary
	this->corrected_scan.mutable_pose()->mutable_position()->set_x(summary.mean.x);
	this->corrected_scan.mutable_pose()->mutable_position()->set_y(summary.mean.y);
	this->corrected_scan.mutable_pose()->mutable_orentation()->set_yaw(summary.mean.theta);

	//robot's pose is the pose of robot base, which may not equal to the laser's pose
	//double lx=-0.6, ly=0.38,lt=2.355;
	//double lx=0.6, ly=-0.38,lt=-0.7854;
	double lx, ly, lt;
	lx = laser_x();
	ly = laser_y();
	lt = laser_theta();
	double robot_t;
	robot_t=summary.mean.theta - lt;
	this->corrected_scan.mutable_robot()->mutable_position()->set_x(
		summary.mean.x - lx*cos(robot_t) + ly*sin(robot_t) );
	this->corrected_scan.mutable_robot()->mutable_position()->set_y(
		summary.mean.y - lx*sin(robot_t) - ly*cos(robot_t) );
	this->corrected_scan.mutable_robot()->mutable_orentation()->set_yaw(robot_t );

	
	unlock();
    return;
}
void processor::set_map( Map &map )
{
	lock();
	this->parameter();
    // free the memory
    if(raw_map.complete_map)
        delete[] raw_map.complete_map;
    if(raw_map.map)
        delete[] raw_map.map;
    raw_map.complete_map = NULL;
    raw_map.map = NULL;
 

    //fill the raw_map with the image
#ifndef NO_ICP
    std::vector<icp::ICP_Point> model;
#endif
	raw_map.config.resolution  = map.resolution();
    raw_map.config.x_size = map.width();
    raw_map.config.y_size = map.height();
    raw_map.config.map_name = NULL;
    raw_map.complete_map = new float[raw_map.config.x_size * raw_map.config.y_size];
    raw_map.map = new float* [raw_map.config.x_size];
    for (int x= 0; x < map.width(); ++x)
    {
        raw_map.map[x] = raw_map.complete_map + x * map.height();
        for(int y = 0;y < map.height(); y++)
        {
            float *p = raw_map.complete_map + x * raw_map.config.y_size + y;
            unsigned int v = map(x,y);
			if(v != Map::UNKOWN)
				*p = 1.0 - Map::red(v) / 255.0;
			else
				*p = -1;
#ifndef NO_ICP
            if(*p > 0.2)
            {
                icp::ICP_Point point( x * map.resolution(), y * map.resolution());
                model.push_back(point);
            }
#endif
        }
    }
    std::cerr<<"localize map set: "<<raw_map.config.x_size
            <<" X "<<raw_map.config.y_size
            << " @ " << raw_map.config.resolution<< std::endl;
#ifndef NO_ICP
    icp_processor.set_model(model);
#endif
    to_localize_map(&raw_map, &pfmap, &param);
	unlock();
}

double processor::get_laser_envir_matching_degree(){
	return summary.mean_scan[summary.num_readings].prob / (double)summary.num_readings;
}

double processor::evaluate_result(){
	stdmsg::Laser_Scan scan = this->corrected_scan;
	float angle = scan.config().angle_min() + scan.pose().orentation().yaw();
	float angle_increment = scan.config().angle_increment();
	float laserX = scan.pose().position().x();
	float laserY = scan.pose().position().y();
	int size = scan.ranges_size();
	int resolution = this->pfmap.config.resolution;
	int x_bound = this->pfmap.config.x_size;
	int y_bound = this->pfmap.config.y_size;
	double evaluate = 0;
	for (int i = 0; i < size; ++i)
	{
		float r = scan.ranges(i);
		if (r < scan.config().range_max() - 0.01)
		{
			float _x = cos(angle) * r + laserX;
			float _y = sin(angle) * r + laserY;
			int x = _x / resolution;
			int y = _y / resolution;
			if (x < 0 || y < 0 || x >= x_bound || y >= y_bound || this->pfmap.map.map[x][y] == -1){
				continue;
			}
			double prob = this->pfmap.prob[x][y];
			if (prob > 0.5){
				evaluate += 1;
			}
		}
		angle += angle_increment;
	}
	return evaluate / size;
}