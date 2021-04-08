#include "planner.h"

#define NUM_ACTIONS 8
int planner_x_offset[NUM_ACTIONS] = { 0, 1, 1, 1, 0,-1,-1,-1};
int planner_y_offset[NUM_ACTIONS] = {-1,-1, 0, 1, 1, 1, 0,-1};

inline double distance_traj(traj_point_p p1, traj_point_p p2)
{
  return sqrt((p1->x-p2->x)*(p1->x-p2->x) + (p1->y-p2->y)*(p1->y-p2->y));
}

inline int
map_to_world(map_point_p map_point,
            world_point_p world_point)
{
  double x = map_point->x * map_point->map->config.resolution;
  double y = map_point->y * map_point->map->config.resolution;

  world_point->pose.x = x;
  world_point->pose.y = y;
  world_point->pose.theta = 0;
  world_point->map = map_point->map;

  return 0;
}
inline void
map_destroy(map_p *map)
{
  free((*map)->complete_map);
  free((*map)->map);
  free((*map));
  *map = NULL;
}
inline int
world_to_map(world_point_p world_point,
            map_point_p map_point)
{
  int x =
    round(world_point->pose.x / world_point->map->config.resolution);
  int y =
    round(world_point->pose.y / world_point->map->config.resolution);

  map_point->x = x;
  map_point->y = y;

  map_point->map = world_point->map;

  return 0;
}
#ifdef _WIN32
#if (_MSC_VER < 1800)
inline int round(double X)
{
    if (X >= 0)
        return (int)(X + 0.5);
    else
        return (int)(X - 0.5);
}
#endif
#endif
inline int
trajectory_to_map(traj_point_p traj_point,
                         map_point_p map_point,
                         map_p map)
{
    int x = round(traj_point->x / map->config.resolution);
    int y = round(traj_point->y / map->config.resolution);

    map_point->x = x;
    map_point->y = y;

    map_point->map = map;

    return 0;
}
inline int
map_to_trajectory(map_point_p map_point,
                         traj_point_p traj_point)
{
    double x = map_point->x * map_point->map->config.resolution;
    double y = map_point->y * map_point->map->config.resolution;

    traj_point->x = x;
    traj_point->y = y;
    traj_point->theta = 0.0;
    traj_point->t_vel = 0.0;
    traj_point->r_vel = 0.0;

    return 0;
}

inline map_p map_copy(map_p map)
{
    map_p new_map;
    int i;

    new_map=(map_p)calloc(1, sizeof(map_t));
    assert(new_map);

    *new_map = *map;
    new_map->complete_map=(float *)
            calloc(map->config.x_size*map->config.y_size, sizeof(float));
    assert(new_map->complete_map);
    memcpy(new_map->complete_map, map->complete_map,
           sizeof(float)*map->config.x_size*map->config.y_size);

    new_map->map=(float **)calloc(map->config.x_size, sizeof(float *));
    assert(new_map->map);

    for (i = 0; i < new_map->config.x_size; i++)
        new_map->map[i] = new_map->complete_map+i*new_map->config.y_size;

	//crti:2016-12-20,navmap
	new_map->complete_navmap=(float *)
            calloc(map->config.x_size*map->config.y_size, sizeof(float));
    assert(new_map->complete_navmap);
    memcpy(new_map->complete_navmap, map->complete_navmap,
           sizeof(float)*map->config.x_size*map->config.y_size);

    new_map->navmap=(float **)calloc(map->config.x_size, sizeof(float *));
    assert(new_map->navmap);

    for (i = 0; i < new_map->config.x_size; i++)
        new_map->navmap[i] = new_map->complete_navmap+i*new_map->config.y_size;

    return new_map;
}


map_t *planner_map = NULL;

static map_t *true_map = NULL;

static int have_plan = 0;

static int allow_any_orientation = 0;
static point_t requested_goal;
static point_t intermediate_goal;
static int goal_is_accessible;

static traj_point_t robot;
static planner_path_t path = {NULL, 0, 0};

static int goal_set = 0;

static int extract_path_from_value_function(void) 
{
    int position;
    traj_point_t path_point;
    map_point_t cur_point, prev_point, map_goal;

    if (!have_plan)
        return -1;

    planner_util_clear_path(&path);
    planner_util_add_path_point(robot, &path);

    trajectory_to_map(&robot, &cur_point, planner_map);

    if (goal_is_accessible) {
        map_goal.x = round(requested_goal.x /
                                  planner_map->config.resolution);
        map_goal.y = round(requested_goal.y /
                                  planner_map->config.resolution);
        map_goal.map = planner_map;
    } else {
        map_goal.x = round(intermediate_goal.x /
                                  planner_map->config.resolution);
        map_goal.y = round(intermediate_goal.y /
                                  planner_map->config.resolution);
        map_goal.map = planner_map;
    }
    do {
        prev_point = cur_point;
        conventional_find_best_action(&cur_point);
        map_to_trajectory(&cur_point, &path_point);
        position = planner_util_add_path_point(path_point, &path);
        if (cur_point.x == map_goal.x && cur_point.y == map_goal.y)
            return 0;
    } while (cur_point.x != prev_point.x || cur_point.y != prev_point.y);

    return -1;
}

static void compute_cost(int start_index, int end_index, double *cost_along_path,
             double *min_cost)
{
    traj_point_p start_point;
    traj_point_p end_point;

    map_point_t p1;
    map_point_t p2;

    bresenham_param_t params;

    int x, y;
    double total_cost, cur_cost = 0;

    start_point = planner_util_get_path_point(start_index, &path);
    trajectory_to_map(start_point, &p1, planner_map);
    end_point = planner_util_get_path_point(end_index, &path);
    trajectory_to_map(end_point, &p2, planner_map);

    get_bresenham_parameters(p1.x, p1.y, p2.x, p2.y, &params);

    total_cost = 0;
    get_current_point(&params, &x, &y);
    *min_cost = conventional_get_cost(x, y);
    while (get_next_point(&params)) {
        get_current_point(&params, &x, &y);
        cur_cost = conventional_get_cost(x, y);
        total_cost += cur_cost;
        if (cur_cost < *min_cost)
            *min_cost = cur_cost;
    }

    *cost_along_path = total_cost;
}

static double cost_of_path(void)
{
    int path_index;
    double cost, min_cost;
    double total_cost;

    total_cost = 0.0;
    for (path_index = 1; path_index < path.length;
         path_index++)
    {
        compute_cost(path_index-1, path_index, &cost, &min_cost);
        total_cost += cost;
    }

    return total_cost;
}


static void smooth_path(navigator_config_t *nav_conf )
{
    int path_index;
    int new_path_counter;
    double cost_along_prev, cost_along_next;
    double min_cost_prev, min_cost_next;
    double new_cost, new_min_cost;
    double prev_cost;

    prev_cost = cost_of_path();

    new_path_counter = 0;
    path_index = 1;

    while (path.length > 2 && distance_traj
           (&robot, planner_util_get_path_point(1, &path)) <
           nav_conf->goal_size) {
        planner_util_delete_path_point(1, &path);
    }

    while (path_index < path.length-1)
    {
        compute_cost(path_index-1, path_index, &cost_along_prev, &min_cost_prev);
        compute_cost(path_index, path_index+1, &cost_along_next, &min_cost_next);
        compute_cost(path_index-1, path_index+1, &new_cost, &new_min_cost);

		if (cost_along_prev+cost_along_next+nav_conf->smooth_threshold < new_cost ||
                min_cost_next < new_min_cost || min_cost_prev < new_min_cost)
        {
            path_index++;
        }
        else
        {
            planner_util_delete_path_point(path_index, &path);
        }
    }
    new_cost = cost_of_path();
}

static int find_nearest_free_point_to_goal(void)
{
    world_point_t goal_world;
    double *util_ptr;
    int x, y;
    double closest_free_dist;
    map_point_t closest_free;
    double dist;
    int goal_x, goal_y;

    goal_x = round(robot.x / planner_map->config.resolution);
    goal_y = round(robot.y / planner_map->config.resolution);

    conventional_dynamic_program(goal_x, goal_y);

    util_ptr = conventional_get_utility_ptr();

    if (util_ptr == NULL) {
        return 0;
    }

    goal_x = round(requested_goal.x /
                          planner_map->config.resolution);
    goal_y = round(requested_goal.y /
                          planner_map->config.resolution);

    closest_free_dist = MAXDOUBLE;
    closest_free.map = planner_map;
    for (x = 0; x < planner_map->config.x_size; x++)
        for (y = 0; y < planner_map->config.y_size; y++) {
            dist = hypot(x-goal_x, y-goal_y);
            if (*util_ptr >= 0 && dist < closest_free_dist) {
                closest_free.x = x;
                closest_free.y = y;
                closest_free_dist = dist;
            }
            util_ptr++;
        }

    if (closest_free_dist > MAXDOUBLE/2) {
        return 0;
    }

    conventional_dynamic_program(closest_free.x, closest_free.y);

    map_to_world(&closest_free, &goal_world);
    intermediate_goal.x = goal_world.pose.x;
    intermediate_goal.y = goal_world.pose.y;

    return 1;
}

static void plan(navigator_config_t *nav_conf)
{
    static int old_goal_x, old_goal_y;
    static traj_point_t old_robot;
    static map_point_t map_pt;
    int goal_x, goal_y;


    goal_x = round(requested_goal.x /
                          planner_map->config.resolution);
    goal_y = round(requested_goal.y /
                          planner_map->config.resolution);

    conventional_dynamic_program(goal_x , goal_y);

    trajectory_to_map(&robot, &map_pt, planner_map);

    if (conventional_get_utility(map_pt.x, map_pt.y) < 0) {
        goal_is_accessible = 0;
        if (nav_conf->plan_to_nearest_free_point)
            have_plan = find_nearest_free_point_to_goal();
    } else {
        have_plan = 1;
        goal_is_accessible = 1;
    }

    if (conventional_get_utility(map_pt.x, map_pt.y) < 0) {
        goal_is_accessible = 0;
        if (nav_conf->plan_to_nearest_free_point)
            have_plan = find_nearest_free_point_to_goal();
    } else {
        have_plan = 1;
        goal_is_accessible = 1;
    }

    if (conventional_get_utility(map_pt.x, map_pt.y) < 0) {
        goal_is_accessible = 0;
        if (nav_conf->plan_to_nearest_free_point)
            have_plan = find_nearest_free_point_to_goal();
    } else {
        have_plan = 1;
        goal_is_accessible = 1;
    }

    if (conventional_get_utility(map_pt.x, map_pt.y) < 0) {
        goal_is_accessible = 0;
        if (nav_conf->plan_to_nearest_free_point)
            have_plan = find_nearest_free_point_to_goal();
    } else {
        have_plan = 1;
        goal_is_accessible = 1;
    }

    old_goal_x = goal_x;
    old_goal_y = goal_y;
    old_robot = robot;
}

//we need to make regenerate trajectory polymorphic somehow
static void regenerate_trajectory(navigator_config_t *nav_conf)
{
    //struct timeval start, end;
    //int sec, msec;
    int index;
    traj_point_p path_point;

    //gettimeofday(&start, NULL);
    if (extract_path_from_value_function() < 0)
        planner_util_clear_path(&path);

    else  {
        if (nav_conf->smooth_path){
            smooth_path(nav_conf);
        }

        /* Add path orientations in */
        index = 1;
        while (index < path.length-1) {
            path.points[index].theta = atan2(path.points[index+1].y-path.points[index].y,path.points[index+1].x-path.points[index].x);
            index++;
        }

        if (path.length > 1) {
            if (!goal_is_accessible || allow_any_orientation)path.points[path.length-1].theta = path.points[path.length-2].theta;
            else path.points[path.length-1].theta = requested_goal.theta;
        }

        /* End of adding path orientations */

        //gettimeofday(&end, NULL);
        //msec = end.tv_usec - start.tv_usec;
        //sec = end.tv_sec - start.tv_sec;
        //if (msec < 0) {
        //    msec += 1e6;
        //    sec--;
        //}
        for (index = 0; index < path.length; index++) {
            path_point = planner_util_get_path_point(index, &path);
        }
    }
}

int planner_update_robot(traj_point_p new_position,
                            navigator_config_t *nav_conf)
{
    static traj_point_t old_position;
    static int first_time = 1;

    if (!planner_map)
        return 0;

    if (new_position->x < 0 || new_position->y < 0 ||
            new_position->x >
            planner_map->config.resolution*
            planner_map->config.x_size ||
            new_position->y >
            planner_map->config.resolution*planner_map->config.y_size)
        return 0;

    robot = *new_position;

    if (!first_time && distance_traj(new_position, &old_position) <
            planner_map->config.resolution)
        return 0;
    regenerate_trajectory(nav_conf);
    old_position = *new_position;

    return 1;
}

int planner_update_goal(point_p new_goal, int any_orientation,
                           navigator_config_t *nav_conf )
{
    if (!planner_map)
        return 0;

    requested_goal = *new_goal;
    allow_any_orientation = any_orientation;
    goal_set = 1;

    plan(nav_conf);
    regenerate_trajectory(nav_conf);

    return 1;
}

void planner_set_map(map_p new_map, float robotWidth )
{
    planner_map = new_map;

    if (true_map != NULL)
        map_destroy(&true_map);

    true_map = map_copy(planner_map);

    map_modify_clear(true_map, planner_map);
    conventional_build_costs( robotWidth,NULL, NULL);
}


void planner_reset_map(float robotWidth)
{  
    map_modify_clear(true_map, planner_map);
    conventional_build_costs(robotWidth, NULL, NULL);
}

void planner_update_map(robot_laser_message *laser_msg,
                          navigator_config_t *nav_conf)
{
    world_point_t world_point;
    map_point_t map_point;

    if (planner_map == NULL)
        return;

    world_point.pose.x = laser_msg->laser_pose.x;
    world_point.pose.y = laser_msg->laser_pose.y;
    world_point.pose.theta = laser_msg->laser_pose.theta;
    world_point.map = planner_map;
    //printf(".");
    map_modify_update(laser_msg, nav_conf, &world_point, true_map, planner_map);
    //printf(",");
    world_to_map(&world_point, &map_point);
    conventional_build_costs(nav_conf->robot_width, &map_point, nav_conf);

    if (!goal_set)
        return;
    //printf("0");
    plan(nav_conf);
    regenerate_trajectory(nav_conf);
}

void planner_get_status(planner_status_p status)
{
    int index;
    status->goal = requested_goal;
    status->robot = robot;
    status->goal_set = goal_set;
    status->path.length = path.length;
    if (status->path.length > 0) {
        status->path.points = (traj_point_p)
                calloc(status->path.length, sizeof(traj_point_t));
        assert(status->path.points);
        for (index = 0; index < status->path.length; index++)
            status->path.points[index] = path.points[index];
    } else  {
        status->path.points = NULL;
    }
    return;
}


