#include "route_planner.h"

using namespace std;

void clip(double &val, double bound)
{
	if (val > bound){
		val = bound;
	}
	else if (val < -bound){
		val = -bound;
	}
}

double calc_dist(double x, double y)
{
	return sqrt(x * x + y * y);
}

inline double clip_theta(double theta){
	double t = theta;
	while (t > 3.14)
		t -= 2 * 3.1415926;
	while (t < -3.14)
		t += 2 * 3.1415926;
	return t;
}

Planner::Planner(middleware::ConfigFile& cfg)
{
		initParam(cfg);
}

void Planner::initParam(middleware::ConfigFile& cfg){
	start = Point(0.0, 0.0);
	target = Point(0.0, 0.0);
	is_pose_initialized = 0;
	is_updating_path = 1;
	posx_ = 0.0;
	posy_ = 0.0;
	the_ = 0.0;
	dist = 0.0;
	delta_orien = 0.0;
	nextx = 0.0;
	nexty = 0.0;
	nextbeta = 0.0;
	goalflag = 1;
	distflag = 1;
	point_p = 0;
	pointsize = 0;
	toward = 1;

	len_th = (double)cfg.value("route_planner", "len_th", 0.8);
	dist_th = (double)cfg.value("route_planner", "dist_th", 0.3);
	orien_th = (double)cfg.value("route_planner", "orien_th", 5);
	rad = (double)cfg.value("route_planner", "rad", 0.2);
	waypoints_file = (string)cfg.value("route_planner", "waypoints_file", string("path.txt"));
	max_v = (double)cfg.value("route_planner", "max_v", 0.3);
	max_w = (double)cfg.value("route_planner", "max_w", 0.2);
		
	linear_coef = (double)cfg.value("route_planner", "linear_coef", 0.4);
	angular_coef_1 = (double)cfg.value("route_planner", "angular_coef_1", 0.07);
	angular_coef_2 = (double)cfg.value("route_planner", "angular_coef_2", 0.245);
	angular_coef_3 = (double)cfg.value("route_planner", "angular_coef_3", 0.03);

	this->logfile.open("debug_helper.txt", std::ios::out | std::ios::app);

	pointVec points;
	ifstream in;
	in.open(waypoints_file);
	while (!in.eof())
	{
		double x, y;
		in >> x >> y;
		points.push_back({ x, y });
	}
	int total_points = points.size();

	printf("Get total points = %d", total_points);
	printf("Generating success");
	in.close();

	astar = Astar(rad, points);
	cmd.set_v(0);
	cmd.set_w(0);
}

/***********************每收到一个新goal就开始一次新的搜索********************************/
void Planner::setGoal(const stdmsg::Pose &goal)
{
	this->is_updating_path = 1;

	while (!is_pose_initialized){ cout << "waiting for initialization" << endl; }
	point_t goal_raw = { goal.position().x(), goal.position().y() };
	point_t goal_nearest;
	goal_nearest = astar.waypoints.nearest_point(goal_raw);
	this->target.x = goal_nearest[0];
	this->target.y = goal_nearest[1];
	this->target_yaw = goal.orentation().yaw();
	std::cout << "start: " << this->start.x << ' ' << this->start.y << endl;
		
	this->planning.clear_path();
	list<Point *> path;
	path = astar.GetPath(this->start, this->target, false);
	if (path.size() == 0){
		std::cout << "the planned path is with size 0" << std::endl;
	}
	for (auto &p: path)
	{
		this->planning.add_path();
		int index = this->planning.path_size() - 1;
		this->planning.mutable_path(index)->mutable_position()->set_x(p->x);
		this->planning.mutable_path(index)->mutable_position()->set_y(p->y);
		this->logfile << "(i-th, x, y)" << "( " << index + 1 << ", " << p->x << ", " << p->y << ") " << endl;
	}
	// path_publisher_.publish(planning);
	this->point_p = 0;
	this->pointsize = planning.path_size();

	if (this->pointsize == 0){
		this->goalflag = 1;
	}
	else{
		this->goalflag = 0;
		this->distflag = 0;
	}

	this->is_updating_path = 0;
}

void Planner::setPose(const stdmsg::Pose &pose)
{
	//为下一次搜寻路径做准备
	//cout << "Loading Pose" << std::endl;
	
	point_t start_raw = { pose.position().x(), pose.position().y() };
	point_t start_nearest;
	start_nearest = astar.waypoints.nearest_point(start_raw);

	this->start.x = start_nearest[0];
	this->start.y = start_nearest[1];

	if (!this->is_pose_initialized){
		this->is_pose_initialized = 1;
		std::cout << "successfully change init pose" << std::endl;
	}

	//为本次跟踪做准备
	//当前amcl计算所得位姿
	this->posx_ = pose.position().x();
	this->posy_ = pose.position().y();
	this->the_ = pose.orentation().yaw();

	/* 设置位姿完了，若还在更新路径，则跳过 */
	if (is_updating_path){ return; }
	if (this->goalflag == 0)  
	{
		findnextpoint();
		calPID();
	}
}

void Planner::findnextpoint()
{
	/*****************重要：下一个目标点不是最近点，始终离当前位置有len_th的距离***********************/
	while (this->point_p < this->pointsize && this->dist < this->len_th){
		this->dist = calc_dist(this->planning.path(this->point_p).position().x() - this->posx_, this->planning.path(this->point_p).position().y() - this->posy_);
		this->nextx = this->planning.path(this->point_p).position().x();
		this->nexty = this->planning.path(this->point_p).position().y();
		if (this->point_p == this->pointsize - 1){
			this->nextbeta = this->target_yaw;
		}
		else
		{
			double yy = this->planning.path(this->point_p + 1).position().y() - this->planning.path(this->point_p).position().y();
			double xx = this->planning.path(this->point_p + 1).position().x() - this->planning.path(this->point_p).position().x();
			this->nextbeta = atan2(yy, xx);
		}
		this->point_p++;
	}
	this->dist = calc_dist(this->nextx - this->posx_, this->nexty - this->posy_);
	this->delta_orien = clip_theta(this->target_yaw - this->the_);
	if (this->point_p == this->pointsize && this->dist < this->dist_th){
		this->logfile << "change distflag" << std::endl;
		this->logfile << "point_p: " << this->point_p << std::endl;
		this->logfile << "point_size:" << this->pointsize << std::endl;
		this->logfile << "dist:" << this->dist << std::endl;
		this->distflag = 1;
	}
	if (this->distflag == 1 && fabs(this->delta_orien) * 180 / 3.1415926 < this->orien_th){
		this->logfile << "change goalflag" << std::endl;
		this->logfile << "delta orien:" << fabs(this->delta_orien) << std::endl;
		this->goalflag = 1;
	}
}

void Planner::calPID()
{
	//boost::mutex::scoped_lock lock(publish_mutex_);
	//cout << "Caculating: " << endl;
	double yy = this->nexty - this->posy_;
	double xx = this->nextx - this->posx_ + 1e-10;
	double theta = atan2(yy, xx);                           //机器人与目标点连线夹角
	double apha = theta - this->the_;                             //机器人初始转角(机器人与连线夹角)
	double beta = this->nextbeta - theta;		                      //机器人第二次转角(连线与目标角度)

	while (apha > 3.14)
		apha -= 2 * 3.1415926;
	while (apha<-3.14)
		apha += 2 * 3.1415926;

	while (beta>3.14)
		beta -= 2 * 3.1415926;
	while (beta < -3.14)
		beta += 2 * 3.1415926;

	double cosa = cos(apha);
	double sina = sin(apha);
	double cos2a = cos(apha / 2.0);

	double v, w;
	
	this->logfile << "now robot is in the pose:: " << "( x: "  << this->posx_ << ", y: " << this->posy_  << ", z:" << this->the_ << ")" << endl;
	this->logfile << "next pose is:: " << "( x: " << this->nextx << ", y: " << this->nexty << ", z: " << this->nextbeta << ")" << endl;
	//this->logfile << "now each PARAM is:: " << "( dist: " << this->dist << ", cosa:" << cosa << ", sina:" << sina << ", cos2a:" << cos2a << ", alpha:" << apha << ", beta" << beta << " )" << endl;;
	this->logfile << "param:: " << "( dist: " << this->dist << ", delta_angle: " << clip_theta(this->nextbeta - this->the_) << endl;
	
	double delta_theta;
	if (this->distflag){
		delta_theta = this->target_yaw - this->the_;
	}
	else{
		if (this->toward){
			delta_theta = theta - this->the_;
		}
		else{
			delta_theta = theta + 3.1415926 - this->the_;
		}
	}

	v = linear_coef * this->dist;
	clip(v, max_v);
	//cmd.linear.x = 0.6*(this->dist+0.10)*cos2a;
	//cmd.linear.x = 0.3*(this->dist+0.10);
	w = angular_coef_2 * clip_theta(delta_theta);
	clip(w, max_w);

	v = v * cos(clip_theta(delta_theta));
	if (fabs(clip_theta(delta_theta)) > angular_coef_1 * 3.141592653 / 180){
		v = 0;
	}
	//w = angular_coef_1 * sina * cosa + angular_coef_2 * apha + (angular_coef_3 * beta * sina * cosa) / apha;
	//cmd.angular.z = 0.11 * sina * cosa + 0.24 * apha;
	//cmd.angular.z = 0.1 * sina * cosa + 0.2 * apha;
	
	if (this->goalflag > 0)
	{
		v = 0;
		w = 0;
	}
	if (this->distflag > 0)
	{
		v = 0;
	}

	if (this->toward == 0)
	{
		v = -v;
	}

	cmd.set_v(v);
	cmd.set_w(w);
	this->logfile << "cmd:: " << "( v:" << v << ", w: " << w << " )" << endl;
	this->logfile << endl;
}

void Planner::setToward(int val){
	this->toward = val;
}

Planner::~Planner(){}

