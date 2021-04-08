
#ifndef CMAP_H
#define CMAP_H


#ifdef __cplusplus
extern "C" {
#endif
#ifdef _WIN32
#define inline __inline
#define M_PI 3.141592653
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <float.h>
#include <math.h>
#include <assert.h>
#ifndef MAXDOUBLE
#define MAXDOUBLE DBL_MAX
#endif
#ifndef MAXFLOAT
#define MAXFLOAT FLT_MAX
#endif

# define M_SQRT2	1.41421356237309504880	/* sqrt(2) */
#define          NAMED_POSITION_TYPE          0
#define          NAMED_POSE_TYPE              1
#define          LOCALIZATION_INIT_TYPE       2

#define          OFFLIMITS_POINT_ID           0
#define          OFFLIMITS_LINE_ID            1
#define          OFFLIMITS_RECT_ID            2

#define          HMAP_LINK_DOOR               1
#define          HMAP_LINK_ELEVATOR           2

typedef struct {
    double x;
    double y;
} position_t, *position_p;


typedef struct {
    double x;
    double y;
    double theta;
} point_t, *point_p;

typedef struct {
    double x;
    double y;
    double theta;
    double t_vel;
    double r_vel;
} traj_point_t, *traj_point_p;

typedef struct {

    double width;
} robot_config_t;

typedef enum { MOTOR, SERVO } arm_joint_t;

typedef struct {
    int X1, Y1;
    int X2, Y2;
    int Increment;
    int UsingYIndex;
    int DeltaX, DeltaY;
    int DTerm;
    int IncrE, IncrNE;
    int XIndex, YIndex;
    int Flipped;
} bresenham_param_t;

typedef void (*usage_func)(char *fmt, ...);

typedef struct {
    int length;
    int capacity;
    int entry_size;
    void *list;
} list_t;

typedef struct {
    double timestamp;
    char *host;
} default_message;

#define DEFAULT_MESSAGE_FMT "{double,string}"

typedef struct {
    char *module_name;
    int pid;
    double timestamp;
    char *hostname;
} heartbeat_message;

typedef struct {
    int x_size;
    int y_size;
    double resolution;
    char *map_name;
} map_config_t, *map_config_p;

typedef struct {
    map_config_t config;
    float* complete_map;
    float** map;
} map_t, *map_p;

typedef struct {
    int type, size;
    char name[22];
    double x, y, theta;
    double x_std, y_std, theta_std;
} place_t, *place_p;

typedef struct {
    double x, y, theta;
} global_offset_t, *global_offset_p;

typedef struct {
    place_p places;
    int num_places;
} map_placelist_t, *map_placelist_p;

typedef struct {
    map_config_t config;
    float *complete_data;
    float ***dist;
} exp_dist_t, *exp_dist_p;

typedef struct {
    int x, y;
    map_p map;
} map_point_t, *map_point_p;

typedef struct {
    point_t pose;
    map_p map;
} world_point_t, *world_point_p;

typedef struct {
    int type;
    int x1, y1, x2, y2;
} offlimits_t, *offlimits_p;

typedef struct {
    offlimits_p offlimits;
    int list_length;
} offlimits_list_t, *offlimits_list_p;

typedef struct {
    double x, y, theta;
    int num_readings;
    float *range;
} laser_scan_t, *laser_scan_p;

/* Heirarchical Map Interface Node (either an elevator or a door connecting two map zones) */
typedef struct {
    int type;                // door or elevator
    int degree;              /* num map zones connected by this link.
                  for doors, this should always equal 2 */
    int *keys;               /* indices into hmap.zone_names[]
                  for elevators, zones are stacked from lowest to highest */
    int num_points;          // 2*degree for doors, 1*degree for elevators
    point_p points;   // indexed by points[key * (num_points / degree) + point]
} hmap_link_t, *hmap_link_p;

typedef struct {
    int num_zones;
    char **zone_names;
    int num_links;
    hmap_link_p links;
} hmap_t, *hmap_p;


typedef struct {
    int num_lasers_to_use;
    //int use_fast_laser;
    double map_update_radius;
    int map_update_obstacles;
    int map_update_freespace;
    //double replan_frequency;
    int smooth_path;
    double waypoint_tolerance;
    double goal_size;
    double goal_theta_tolerance;
    //int dont_integrate_odometry;
    int plan_to_nearest_free_point;
    double robot_width;
} navigator_config_t;

typedef struct {
    traj_point_p points;
    int length;
    int capacity;
} planner_path_t, *planner_path_p;

/** The data structure describing what the motion planning library believes
      is the current robot position, goal and current plan. **/

typedef struct {
    traj_point_t robot;
    point_t goal;
    planner_path_t path;
    int goal_set;
} planner_status_t, *planner_status_p;




/** The laser message of the laser module (rawlaser) **/
typedef struct {
    //laser_laser_type_t  laser_type;  /**< what kind of laser is this **/
    double start_angle;                     /**< angle of the first beam relative **/
    /**< to to the center of the laser **/
    double fov;                             /**< field of view of the laser **/
    double angular_resolution;              /**< angular resolution of the laser **/
    double maximum_range;                   /**< the maximum valid range of a measurement  **/
    //double accuracy;                        /**< error in the range measurements **/
    //laser_remission_type_t remission_mode;  /* if and what kind of remission values are used */

} laser_laser_config_t;
typedef struct {
    int id;
    laser_laser_config_t config;   /**< Configuration of the laser sensor **/
    int num_readings;                     /**< Number of beams in this message **/
    float *range;                         /**< Array of proximity measurements **/
    //char *tooclose;                       /**< Is the robot too close to an obstacle?  **/
    //int num_remissions;                   /**< Number of remission values (0 = no remission available **/
    //float *remission;                    /**< Array of remission measurements **/
    point_t laser_pose;           /**< Position of the center of the laser **/
    point_t robot_pose;           /**< Position of the center of the robot **/
    //double tv, rv;                       /**< Translational and rotational velocities **/
    //double forward_safety_dist, side_safety_dist;
    //double turn_axis;
    //double timestamp;                    /**< Timestamp when the laser data was recorded (received by the devide driver) **/
    //char *host;                          /**< The host from which this message was sent **/
} robot_laser_message;
///*typedef struct {
//    int id;
//    //laser_laser_config_t config;   /**< Configuration of the laser sensor **/
//    int num_readings;                     /**< Number of beams in this message **/
//    float *range;                         /**< Array of proximity measurements **/
//    char *tooclose;                       /**< Is the robot too close to an obstacle?  **/
//    int num_remissions;                   /**< Number of remission values (0 = no remission available **/
//    float *remission;                    /**< Array of remission measurements **/
//    point_t laser_pose;           /**< Position of the center of the laser **/
//    point_t robot_pose;           /**< Position of the center of the robot **/
//    double tv, rv;                       /**< Translational and rotational velocities **/
//    double forward_safety_dist, side_safety_dist;
//    double turn_axis;
//    double timestamp;                    /**< Timestamp when the laser data was recorded (received by the devide driver) **/
//    char *host;                          /**< The host from which this message was sent **/
//} robot_laser_message;


static inline double gaussian_random(double mean, double std)
{
  const double norm = 1.0 / (RAND_MAX + 1.0);
  double u = 1.0 - rand() * norm;                  /* can't let u == 0 */
  double v = rand() * norm;
  double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
  return mean + std * z;
} 
static inline double uniform_random(double min, double max)
{
  return min + (rand() / (double)RAND_MAX) * (max - min);
}
static inline double square(double val)
{
  return (val*val);
}
static inline double normalize_theta(double theta)
{
  double multiplier;

  if (theta >= -M_PI && theta < M_PI)
    return theta;

  multiplier = floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}
static inline  double radians_to_degrees(double theta)
{
  return (theta * 180.0 / M_PI);
}
#ifdef __cplusplus
}
#endif

#endif

