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

	//crti:2016-11-06,改变停止策略，到目标点后只转向，distance_goalreach在设置目标点是改为false
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

	//crti:2016-08-18,把w放到前面
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

	//crti:2016-11-06,距离上到达目标点后的处理
	//线速度设为0，角度误差小于设定值或者因为定位跳变不能小于设定值时
	//认为goalreach，目前不考虑定位跳变
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
		w = error_target * 0.7;//暂时
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
依据当前点和目标点之间的距离、角度，计算出应走的速度和角速度
代替上一个get_best_command()函数
*/
stdmsg::Velocity planner::cal_vw(int is_forward)
{
	this->parameter();
	/* 寻找下一个path中的路径点 */
	const double goal_threadhold = 0.5;
	
	stdmsg::Velocity cmd;
	cmd.set_v(0);
	cmd.set_w(0);
	
	/* 已经在目标位姿了，返回0速度 */
	if (this->_goal_reached == true)
		return cmd;

	/* path中存放的第一个点是当前位姿，所以不需要运动，直接返回 */
	int len = this->global_plan.path_size();	
	if (len <= 1)
		return cmd;

	/* 计算当前点到目标点的距离，如果够近了，就设置distance_goalreach=true */
	double dist2goal = dist(this->robotx - global_plan.path(len - 1).position().x(), this->roboty - global_plan.path(len - 1).position().y());
	if (dist2goal < stop_delta_distance()){
		this->distance_goalreach = true;
	}
	else{
		this->distance_goalreach = false;
	}
	/* 进一步检查角度和目标角度的差异，如果够近了，就设置到达了 */
	bool angleReach = abs(global_plan.path(len - 1).orentation().yaw() - this->robottheta) < stop_delta_theta();
	if (this->distance_goalreach && angleReach){
		this->_goal_reached = true;
		this->global_plan.set_goal_reached(1);
		return cmd;
	}

	/* 从path中找出下一个路径点 */
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

	/* 获取当前位姿与下一个路径点之间的一些数量关系，包括距离、角度等 */
	double target_x = global_plan.path(index).position().x();
	double target_y = global_plan.path(index).position().y();
	double target_theta = global_plan.path(index).orentation().yaw();

	double theta = atan2(target_y - this->roboty, target_x - this->robotx);//机器人与目标点连线夹角
	double apha = theta - this->robottheta;                                //机器人初始转角(机器人与连线夹角)
	double beta = target_theta - theta;	                                   //机器人第二次转角(连线与目标角度)
	double delta_theta = target_theta - this->robottheta;				   //机器人姿态与目标姿态差值
	
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
	
	/* 如果当前位姿和路径点位姿相近，那么返回0速度*/
	if (dist2target < stop_delta_distance() && abs(delta_theta) < stop_delta_theta() * 3.1415926 / 180.0)
	{
		dist2target = 0;
		delta_theta = 0;
		return cmd;
	}
	double v, w;
	/* 如果没有达标，则计算跟踪速度和角速度 */	
	/* 这种情况只可能出现在还剩一个目标点的时候，因为我们要求下一个目标点的距离与当前位置至少有goal_threadhold的距离 */
	/* 正常情况下，就是一个与距离有关的函数 */
	
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

