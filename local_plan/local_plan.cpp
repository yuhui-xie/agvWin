#include "local_plan.h"
#include <vector>
#include <iostream>

using namespace local_plan;
using namespace std;

const unsigned int MIN_COST = 0;
const unsigned int MAX_COST = 0xFF;
inline double dist(double x, double y)
{
    return sqrt(x*x + y*y);
}

planner::planner():
    obstacle(NULL), costmap(NULL),
    robotx(0), roboty(0), robottheta(0), robotsteer(0),
    initialized(false),
    _goal_reached(false)
{
	//crti:2016-11-06
	this->distance_goalreach = false;
}
planner::~planner()
{
    if(this->obstacle)
    {
        delete this->obstacle;
    }
    if(this->costmap)
    {
        delete this->costmap;
    }
}
void planner::reset_map()
{
    for(int ix = 0; ix < this->obstacle->width(); ++ix)
    {
        for(int iy = 0; iy < this->obstacle->height(); ++iy)
        {
            this->obstacle->operator()( ix, iy ) = 0x00000000;
            //this->costmap->operator() (ix, iy ) = 0x00000000;
        };
    }

    //default initialize the origin to the center of Matrix
    origin_x =  this->obstacle->width() / 2;
    origin_y =  this->obstacle->height() / 2;
}
void planner::parameter()
{
    if(initialized)
        return;
    initialized = true;

    int map_size = this->_max_v * this->_period * 2 / this->_space_step;
    this->obstacle = new Map( map_size, map_size, this->_space_step);
    this->costmap = new Map(obstacle->width(), obstacle->height(), obstacle->resolution() );
    this->reset_map();


    /** initialize the particles */
    for( double v = 0; v < this->_max_v; v += this->_v_step)
    {
        for(double w = -this->_max_w; w < this->_max_w; w += this->_w_step)
            this->particles.push_back(velocity(v,w));
    }
#ifdef DEBUG_HELPER
    for(int i = 0; i < this->particles.size(); i ++)
    {
        double cmd_v = this->particles[i].v;
        double cmd_w = this->particles[i].w;


        printf("%f %f %f \n", cmd_v, cmd_w, this->particles[i].weight);

    }
#endif
}
void planner::set_plan( const stdmsg::Global_Plan& plan )
{
    global_plan = plan;
}
void planner::set_velocity(const stdmsg::Velocity &vel)
{
   this->robotv = vel.v();
   this->robotw = vel.w();
}

void planner::update(const stdmsg::Laser_Scan& scan)
{
    this->parameter();
    /* store the current state of the robot */
    this->robotx = scan.robot().position().x();
    this->roboty = scan.robot().position().y();
    this->robottheta  = scan.robot().orentation().yaw();
    this->robotsteer = scan.steer();
    double d = dist(this->robotx - scan.pose().position().x(),
                    this->roboty - scan.pose().position().y() );
    double t = atan2( scan.pose().position().y() - this->roboty,
                      scan.pose().position().x() - this->robotx ) - scan.robot().orentation().yaw();

    double laserx = d * cos( t );
    double lasery = d * sin( t );
    double lasert = scan.pose().orentation().yaw() - scan.robot().orentation().yaw();

	
	//hxw 8.16
    this->reset_map();
    for(int i = 0; i < scan.ranges_size(); i++)
    {
        double angle = scan.config().angle_min() + i * scan.config().angle_increment() + lasert;
        double cos_ = cos(angle);
        double sin_ = sin(angle);
        double ix = ( laserx + scan.ranges(i) * cos_ )  / this->_space_step + origin_x;
        double iy = ( lasery + scan.ranges(i) * sin_ ) /  this->_space_step + origin_y;;
        for(double len = scan.ranges(i); len >= 0;
            len -= this->_space_step, ix -= cos_, iy -= sin_)
        {
            int iix = int(ix+0.5);
            int iiy = int(iy+0.5);
            if( iix >= 0 && iix < this->obstacle->width() &&
                    iiy >= 0 && iiy < this->obstacle->height() )
            {
                if(len > scan.ranges(i) - 0.1 )
                    this->obstacle->operator()( iix, iiy ) = 0x000000FF;
                else
                    this->obstacle->operator()( iix, iiy ) = 0x00000000;
            }
        }
    }

    // build the cost map
    /* Initialize cost function to match map, where empty cells have
    MIN_COST cost and filled cells have MAX_UTILITY cost */
	
    for (int x_index = 0; x_index < costmap->width(); x_index++)
    {
        for (int y_index = 0; y_index < costmap->height(); y_index++)
        {
            if ( this->obstacle->operator()(x_index, y_index) > 0x10 )
                this->costmap->operator()(x_index, y_index) = MAX_COST;
            else
                this->costmap->operator()(x_index, y_index) = MIN_COST;
        }
    }
	
    unsigned int value = MAX_COST - this->costmap->resolution() * MAX_COST;
    /* Loop through cost map, starting at top left, updating cost of cell
    as max of itself and most expensive neighbour less the cost
    downgrade. */
	
    double damping = this->costmap->resolution() * MAX_COST * 2.0;
    const int NUM_ACTIONS = 8;
    int planner_x_offset[NUM_ACTIONS] = {0,   1, 1, 1, 0, -1, -1, -1};
    int planner_y_offset[NUM_ACTIONS] = {-1, -1, 0, 1, 1,  1,  0, -1};
    for (int x_index = 0; x_index < costmap->width(); x_index++)
    {
        for (int y_index = 0; y_index < costmap->height(); y_index++)
        {
            if (x_index < 1 || x_index >= costmap->width() - 1
                    || y_index < 1 || y_index >= costmap->height() -1 )
                continue;

            for (int index = 0; index < NUM_ACTIONS; index++) {
                int x = x_index + planner_x_offset[index];
                int y = y_index + planner_y_offset[index];

                unsigned int value = this->costmap->operator()(x, y) - damping;
                unsigned int& p = this->costmap->operator()(x_index, y_index);
                if ( value > p && value < MAX_COST )
                    p = value;
            }
        }
    }

    for (int x_index = costmap->width() - 1; x_index >= 0; x_index--)
    {
        for (int y_index =  costmap->height() - 1; y_index >= 0; y_index--)
        {
            if (x_index < 1 || x_index >= costmap->width() - 1
                    || y_index < 1 || y_index >= costmap->height() -1 )
                continue;

            for (int index = 0; index < NUM_ACTIONS; index++) {
                int x = x_index + planner_x_offset[index];
                int y = y_index + planner_y_offset[index];

                unsigned int value = this->costmap->operator()(x, y) - damping;
                unsigned int& p = this->costmap->operator()(x_index, y_index);
                if ( value > p  && value < MAX_COST )
                    p = value;
            }
        }
    }


#ifdef DEBUG_HELPER
    std::cout<<costmap->width()<<" "<<costmap->height()<<" "<< costmap->resolution()<<std::endl;
    this->obstacle->save("obstacle.png", false);
    this->costmap->save("costmap.png", false);
#endif
}

double planner::get_distance_obstacle_helper(double x, double y, double theta)
{
    return 100;
    double min_distance = 1e8;
    std::vector<point> poly(this->_footprint);

    int start_x = costmap->width(), end_x = 0;
    int start_y = costmap->height(), end_y = 0;

    double cos_ = cos(theta);
    double sin_ = sin(theta);
    for(int vetex = 0; vetex < poly.size(); vetex++ )
    {
        //translate the poly
        double x_ = poly[vetex].x * cos_ - poly[vetex].y * sin_ + x;
        double y_ = poly[vetex].x * sin_ + poly[vetex].y * cos_ + y;
        x_ = x_ / this->_space_step + origin_x;
        y_ = y_ / this->_space_step + origin_y;
        poly[vetex].x = x_;
        poly[vetex].y = y_;
        if( x_ < start_x )start_x = x_;
        if( x_ > end_x )end_x = x_;
        if( y_ < start_y )start_y = y_;
        if( y_ > end_y )end_y = y_;
    }

    if(start_x < 0)start_x = 0;
    if(end_x > costmap->width())end_x = costmap->width();
    if(start_y < 0)start_y = 0;
    if(end_y > costmap->height())end_y = costmap->height();

#ifdef DEBUG_HELPER
    for(int vetex2 = 0; vetex2 < poly.size(); vetex2++ )
    {
        std::cout << poly[vetex2].x << " " << poly[vetex2].y <<std::endl;
    }
#endif

    for (int pixelY = start_y; pixelY < end_y; pixelY++)
    {
        std::vector<int> nodeX;

        int vetex1 = poly.size() - 1;
        for(int vetex2 = 0; vetex2 < poly.size(); vetex2++ )
        {
            if ( (poly[vetex1].y < pixelY && poly[vetex2].y >= pixelY)
                 ||  (poly[vetex2].y < pixelY && poly[vetex1].y >= pixelY) )
            {
                nodeX.push_back(
                            poly[vetex2].x +
                            (pixelY -  poly[vetex2].y)/( poly[vetex1].y -  poly[vetex2].y)
                            *( poly[vetex1].x -  poly[vetex2].x));
            }
            vetex1 = vetex2;
        }

        std::sort( nodeX.begin(), nodeX.end());

        for (int index = 0; index < nodeX.size(); index += 2)
        {
            if   (nodeX[index  ]>=end_x)
                break;
            if   (nodeX[index+1]> start_x )
            {
                if (nodeX[index  ]< start_x ) nodeX[index  ]=start_x ;
                if (nodeX[index+1]> end_x) nodeX[index+1]=end_x;
                //calcate the cost
                for (int j = nodeX[index]; j < nodeX[index+1]; j++)
                {
                    int value = MAX_COST - this->costmap->operator() (j,pixelY);
                    if( value < min_distance )
                        min_distance = value;
#ifdef DEBUG_HELPER
                    this->obstacle->operator() (j,pixelY) = 0xFF;
#endif
                }
            }
        }
    }

    return min_distance;

}
std::pair<double, double> planner::get_distance_goal_helper(double end_x, double end_y, double end_theta)
{
    std::pair<double, double> ret;
    ret.first = 1e8;        /** this is distance of position */
    ret.second = 0;         /** this is distance of rotation */

    int len = this->global_plan.path_size();
    if(len <= 1)
        return ret;

    /** abstruct the goal */
    const double goal_threadhold = 0.5;//0.4
    int index = 0;
    double d;
    do{
        ++index;
        d = dist( this->global_plan.path(index).position().x() - this->global_plan.path(0).position().x(),
              this->global_plan.path(index).position().y() - this->global_plan.path(0).position().y() );
    }while(index < len - 1 && d < goal_threadhold);

    double error_theta = atan2( global_plan.path(index).position().y() - end_y, global_plan.path(index).position().x() - end_x );
    error_theta = error_theta - end_theta;
    double error_distance = dist(end_x - global_plan.path(index).position().x(),
                end_y - global_plan.path(index).position().y() );
	//double error_target = global_plan.path(index).orentation().yaw() - global_plan.path(0).orentation().yaw();
    double error_target = global_plan.path(index).orentation().yaw() - end_theta;

	//printf("++++++%d\t%.3f\t%.3f\n", index,global_plan.path(index).orentation().yaw(),global_plan.path(0).orentation().yaw());
    ret.first = error_distance;
    ret.second = error_theta;
    return ret;
}
double planner::get_distance_path_helper(double end_x, double end_y)
{
    int len = this->global_plan.path_size();
    double distance = 1e8;
    for(int i = 1; i < len; i++)
    {
        double a = dist(end_x - global_plan.path(i).position().x(),
                        end_y - global_plan.path(i).position().y());
        double b = dist(end_x - global_plan.path(i - 1).position().x(),
                        end_y - global_plan.path(i - 1).position().y());
        double c = dist( global_plan.path(i).position().x() - global_plan.path(i - 1).position().x(),
                         global_plan.path(i - 1).position().y() - global_plan.path(i).position().y());

        if( a*a > b*b + c*c || b*b > c*c + a*a )
        {
            double tmp = (a>b?a:b);
            if(tmp < distance )
                distance = tmp;
        }
        else{
            double p = (a + b + c) / 2.0;
            double S = sqrt( p * (p - a) * (p - b) *(p - c) );
            double tmp = S / c;
            if(tmp < distance )
                distance = tmp;
        }
    }
    return distance;
}
double planner::get_comfort_helper(double cmdv, double cmdw)
{
    //double s = 0.2 * exp()
    return 0;
}
bool planner::justify_reached_helper(double x, double y, double t)
{
    int len = this->global_plan.path_size();
    if(len <= 1)
        return false;
    else if( dist(x - global_plan.path(len - 1).position().x(),
                y - global_plan.path(len - 1).position().y() ) < stop_delta_distance() )
        return true;
    return false;
}

stdmsg::Velocity planner::get_best_command()
{
    this->parameter();
    stdmsg::Velocity cmd;
    cmd.set_v(0);
    cmd.set_w(0);

	//crti:2016-11-06,�ı�ֹͣ���ԣ���Ŀ����ֻת��distance_goalreach������Ŀ����Ǹ�Ϊfalse
	if(justify_reached_helper(this->robotx, this->roboty, this->robottheta))
	{
		this->distance_goalreach = true;
	}
	else
	{
		this->distance_goalreach = false;
	}
	

    /** when there is no path to follow or the goal is reached
      * stop the robot
      */
    /* for agv, just tempory */
    std::pair<double, double> ret = get_distance_goal_helper(this->robotx, this->roboty, this->robottheta);

	double distance_to_goal = ret.first;
	double error_theta = ret.second;
    while(error_theta > 3.141592653)
        error_theta -= 3.141592653 * 2;
    while(error_theta < -3.141592653)
        error_theta += 3.141592653 * 2;
    double w;
    double v;

	//crti:2016-08-18,��w�ŵ�ǰ��
	w = error_theta * 0.7;
	v = 0.5 * distance_to_goal;
	if (distance_to_goal > 0.5 && distance_to_goal < 2)
	{
		v = nav_v();
	}
	else if (distance_to_goal > stop_delta_distance() && distance_to_goal < 0.5)
	{
		v = nav_v() / 1.5;
	}
	else if (distance_to_goal < stop_delta_distance())//crti:2016-04-17
	{
		v = 0.0;
		//crti:2016-08-18
	}
	if (fabs(error_theta) > stop_delta_theta() * 3.141592653 / 180 * 2.5)
	{
		v = 0;
	}

	//crti:2016-11-06,�����ϵ���Ŀ����Ĵ���
	//���ٶ���Ϊ0���Ƕ����С���趨ֵ������Ϊ��λ���䲻��С���趨ֵʱ
	//��Ϊgoalreach��Ŀǰ�����Ƕ�λ����
	if(this->distance_goalreach)
	{
		v = 0.0;
		double error_target = 0.0;
		int len = this->global_plan.path_size();
		error_target = global_plan.path(len-1).orentation().yaw() - this->robottheta;
		while(error_target > 3.141592653)
			error_target -= 3.141592653 * 2;
		while(error_target < -3.141592653)
			error_target += 3.141592653 * 2;
		w = error_target * 0.7;//��ʱ
		if(abs(error_target) < stop_delta_theta() * 3.1415926 / 180.0)
		{
			this->_goal_reached = true;
			this->global_plan.set_goal_reached(1);

		}
		else
		{
			this->_goal_reached = false;
			this->global_plan.set_goal_reached(0);
		}
		std::cout << "stop_theta: " << stop_delta_theta() * 3.1415926 / 180.0 << std::endl;
		std::cout << "error_target: " << error_target << std::endl;
	}

    if(v > nav_v())
        v = nav_v();
	if (w > nav_w())
		w = nav_w();
	if (w < -nav_w())
		w = -nav_w();
	
    cmd.set_v(v);
    cmd.set_w(w);
    return cmd;
}

/*
���ݵ�ǰ���Ŀ���֮��ľ��롢�Ƕȣ������Ӧ�ߵ��ٶȺͽ��ٶ�
������һ��get_best_command()����
*/
stdmsg::Velocity planner::cal_vw(int is_forward)
{
	this->parameter();
	/* Ѱ����һ��path�е�·���� */
	const double goal_threadhold = 0.5;
	
	stdmsg::Velocity cmd;
	cmd.set_v(0);
	cmd.set_w(0);
	
	/* �Ѿ���Ŀ��λ���ˣ�����0�ٶ� */
	if (this->_goal_reached == true)
		return cmd;

	/* path�д�ŵĵ�һ�����ǵ�ǰλ�ˣ����Բ���Ҫ�˶���ֱ�ӷ��� */
	int len = this->global_plan.path_size();	
	if (len <= 1)
		return cmd;

	/* ���㵱ǰ�㵽Ŀ���ľ��룬��������ˣ�������distance_goalreach=true */
	double dist2goal = dist(this->robotx - global_plan.path(len - 1).position().x(), this->roboty - global_plan.path(len - 1).position().y());
	if (dist2goal < stop_delta_distance()){
		this->distance_goalreach = true;
	}
	else{
		this->distance_goalreach = false;
	}
	/* ��һ�����ǶȺ�Ŀ��ǶȵĲ��죬��������ˣ������õ����� */
	bool angleReach = abs(global_plan.path(len - 1).orentation().yaw() - this->robottheta) < stop_delta_theta();
	if (this->distance_goalreach && angleReach){
		this->_goal_reached = true;
		this->global_plan.set_goal_reached(1);
		return cmd;
	}

	/* ��path���ҳ���һ��·���� */
	double dist2target;
	int index;
	for (index = 1; index < len - 1; index++)
	{
		dist2target = dist(this->global_plan.path(index).position().x() - this->global_plan.path(0).position().x(),
			this->global_plan.path(index).position().y() - this->global_plan.path(0).position().y());
		if (dist2target < goal_threadhold){
			break;
		}
	}

	/* ��ȡ��ǰλ������һ��·����֮���һЩ������ϵ���������롢�Ƕȵ� */
	double target_x = global_plan.path(index).position().x();
	double target_y = global_plan.path(index).position().y();
	double target_theta = global_plan.path(index).orentation().yaw();

	double theta = atan2(target_y - this->roboty, target_x - this->robotx);//��������Ŀ������߼н�
	double apha = theta - this->robottheta;                                //�����˳�ʼת��(�����������߼н�)
	double beta = target_theta - theta;	                                   //�����˵ڶ���ת��(������Ŀ��Ƕ�)
	double delta_theta = target_theta - this->robottheta;				   //��������̬��Ŀ����̬��ֵ
	
	while (apha>3.14)
		apha -= 2 * 3.1415926;
	while (apha<-3.14)
		apha += 2 * 3.1415926;

	while (beta>3.14)
		beta -= 2 * 3.1415926;
	while (beta<-3.14)
		beta += 2 * 3.1415926;

	double cosa = cos(apha);
	double sina = sin(apha);
	double cos2a = cos(apha / 2.0);
	
	std::cout << "angles:" << cosa << '\t' << sina << '\t' << cos2a << std::endl;
	std::cout << "dist:" << dist2target << std::endl;;
	
	/* �����ǰλ�˺�·����λ���������ô����0�ٶ�*/
	if (dist2target < stop_delta_distance() && abs(delta_theta) < stop_delta_theta() * 3.1415926 / 180.0)
	{
		dist2target = 0;
		delta_theta = 0;
		return cmd;
	}
	double v, w;
	/* ���û�д�꣬���������ٶȺͽ��ٶ� */	
	/* �������ֻ���ܳ����ڻ�ʣһ��Ŀ����ʱ����Ϊ����Ҫ����һ��Ŀ���ľ����뵱ǰλ��������goal_threadhold�ľ��� */
	/* ��������£�����һ��������йصĺ��� */
	
	if (dist2target >= goal_threadhold)
		v = this->v_para() * goal_threadhold * cos2a;
	else
		v = this->v_para() * dist2target * cos2a;
	w = this->w_para_1() * sina * cosa + this->w_para_2() * apha + (this->w_para_3() * beta * sina * cosa) / apha;
	


	if (v > nav_v())
		v = nav_v();
	if (v < -nav_v())
		v = -nav_v();
	if (w > nav_w())
		w = nav_w();
	if (w < -nav_w())
		w = -nav_w();
	cmd.set_v(v);
	cmd.set_w(w);
	return cmd;
}

