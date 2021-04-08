#ifndef LOCAL_PLAN_H
#define LOCAL_PLAN_H
#include <vector>
#include <image.h>
#include <stdmsg.hh>
#include <node.hpp>
#include <math.h>
/** this planner not spport multithread
 * the weight = alpha * distance_to_goal
 *              + belta * distance_to_path
 *              + garmma * distance_to_obstacle
 */
extern bool rotate_flag;
namespace local_plan
{

struct velocity
{
	double v;
	double w;
	double weight;
	velocity(double _v = 0, double _w = 0):v(_v), w(_w), weight(0){}
};
struct point
{
	point(double _x = 0, double _y = 0):x(_x), y(_y){}
	double x;
	double y;
};

class planner
{
public:
    planner();
    ~planner();
    void update(const stdmsg::Laser_Scan& laser_scan);
    void set_plan( const stdmsg::Global_Plan& plan );
    void set_velocity(const stdmsg::Velocity &vel);
    void reset();
	stdmsg::Velocity get_best_command();
	//crti:2016-11-06
	bool distance_goalreach;
private:
	void reset_map();
private:
	/* local_plan中速度规划cal_vw()的参数，反正我也不知道有什么含义，调参就是了~ */
	PARAMETER(double, w_para_1);
	PARAMETER(double, w_para_2);
	PARAMETER(double, w_para_3);
	PARAMETER(double, v_para);
    /* the parameter of the planner */
    PARAMETER(double, w1);      /** safty */
    PARAMETER(double, w2);      /** comfort */
    PARAMETER(double, w3);      /** obedience */
    /** for satety */
    PARAMETER(double, alpha);
    PARAMETER(double, belta1);
    PARAMETER(double, belta2);
	PARAMETER(double, garmma);
	PARAMETER(double, delta);
	
	PARAMETER(double, period);
	PARAMETER(double, t_step);
	PARAMETER(double, v_step);
	PARAMETER(double, w_step);
	PARAMETER(double, space_step);
	
	PARAMETER(double, max_v);
	PARAMETER(double, max_w);
	PARAMETER(double, max_acc_v);
	PARAMETER(double, max_acc_w);

	//crti:2016-05-06
	PARAMETER(double, stop_delta_distance);
	PARAMETER(double, stop_delta_theta);
	PARAMETER(double, nav_v);
	PARAMETER(double, nav_w);

    /* minimal obstacle distacle allowed */
    PARAMETER(double, obstacle_allowed);
    PARAMETER(std::vector<point>, footprint);


	bool initialized;
	void parameter();

    /* the obstacle try to avoid */
	Map* costmap;
	Map* obstacle;
	int origin_x;
	int origin_y;

    /* the global plan try to follow */
    PARAMETER(bool,goal_reached);
	stdmsg::Global_Plan global_plan;

    /* robot current state
     * including pose and velocity */
	double robotx;
	double roboty;
	double robottheta;
    double robotsteer;
	double robotv;
	double robotw;

	bool back_flag;



	std::vector<velocity> particles;
	double get_distance_obstacle_helper(double x, double y, double t);
	double get_distance_path_helper(double x, double y);
    std::pair<double, double> get_distance_goal_helper(double x, double y, double t);
    double get_comfort_helper(double cmdv, double cmdw);
    bool justify_reached_helper(double x, double y, double t);
	stdmsg::Velocity planner::cal_vw(int is_forward);
	stdmsg::Velocity planner::cal_vw_old_way(int is_forward);
};

}
#endif
