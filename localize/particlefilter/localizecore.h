


#ifndef CARMEN_LOCALIZECORE_H
#define CARMEN_LOCALIZECORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "localize_motion.h"
#include "data.h"
#define      SMALL_PROB        0.01
#define      MAX_BEAMS_PER_SCAN   4096

/* #define      LOCALIZECORE_TRACKING_MINLIKELIHOOD        (0.5) */
/* #define      LOCALIZECORE_GLOBAL_MINLIKELIHOOD          (0.9) */

/** localize parameter structure **/
typedef struct {
    double front_laser_offset, rear_laser_offset;
    //double laser_slide_offset;
    //double laser_front_offset;
    int num_particles;
    double max_range, min_wall_prob, outlier_fraction;
    double update_distance;
    double update_angle;
    double integrate_angle;             /**< used to compute laser_skip **/
    int laser_skip;
    int use_rear_laser, do_scanmatching;
    int constrain_to_map;
#ifdef OLD_MOTION_MODEL
    double odom_a1, odom_a2, odom_a3, odom_a4;
#endif
    double occupied_prob;
    double lmap_std;
    double global_lmap_std, global_evidence_weight, global_distance_threshold;
    int global_test_samples;
    int use_sensor;

    double tracking_beam_minlikelihood;
    double global_beam_minlikelihood;

#ifndef OLD_MOTION_MODEL
    localize_motion_model_t *motion_model;
#endif  
} localize_param_t, *localize_param_p;

typedef struct {
    float x, y, theta, weight;
} localize_particle_t, *localize_particle_p;

typedef struct {
    int initialized, first_odometry, global_mode;
    localize_param_p param;
    localize_particle_p particles;
    point_t last_odometry_position;
    float **temp_weights;
    float distance_travelled;
    float angle_rotated;
    char laser_mask[MAX_BEAMS_PER_SCAN];
} localize_particle_filter_t, *localize_particle_filter_p;

typedef struct {
    float x, y, range;
    float prob;
    char mask;
} localize_laser_point_t, *localize_laser_point_p;

typedef struct {
    point_t mean, std;
    point_t odometry_pos;
    double xy_cov;
    int converged;
    int num_readings;
	float match;
    localize_laser_point_t mean_scan[MAX_BEAMS_PER_SCAN];
} localize_summary_t, *localize_summary_p;

#include "likelihood_map.h"


/** Create (allocate memory for) a new particle filter **/
localize_particle_filter_p
localize_particle_filter_new(localize_param_p param);

/** Creates a distribution of particles over the map based on the given observation 
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param laser Laser message used to generate the distribution.
 *  @param map Map which is used to compute p(z|m,x) for the initialized samples.
**/
void 
localize_initialize_particles_uniform(localize_particle_filter_p filter,
                                      robot_laser_message *laser,
                                      localize_map_p map);

/** Creates a multi Gaussian distribution of particles 
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param num_modes Number of modes if Gaussian to create the initial distrubution.
 *  @param mean Array of means (array size = num_modes)
 *  @param std Array of standard variances (array size = num_modes)
**/
void 
localize_initialize_particles_gaussians(localize_particle_filter_p filter,
                                        int num_modes,
                                        point_t *mean,
                                        point_t *std);

/** Creates a Gaussian distribution of particles 
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param mean mean of the Gaussian
 *  @param std std var of the Gaussian
**/
void 
localize_initialize_particles_gaussian(localize_particle_filter_p filter,
                                       point_t mean,
                                       point_t std);


/** Directly initialized the Samples
 *
 *  @param filter Particle filter structure the function is applied to.
**/
void 
localize_initialize_particles_manual(localize_particle_filter_p filter,
                                     double *x,
                                     double *y,
                                     double *theta,
                                     double *weight,
                                     int num_particles);

int 
localize_initialize_particles_placename(localize_particle_filter_p filter,
                                        map_placelist_p placelist,
                                        char *placename);

/** Draw the pose of the samples based on the proposal given by the motion model
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param odometry_position Odometry-based pose estimate.
**/
void 
localize_incorporate_odometry_diff(localize_particle_filter_p filter,
                              point_t odometry_position);

//crti:2016-07-16,diff和omni两种模型
void 
localize_incorporate_odometry_omni(localize_particle_filter_p filter,
                              point_t odometry_position);


/** Compute the particle weights according to the observation likelihood p(z|m,x)
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param map Map of the environment.
 *  @param num_readings Number of beams of that measurement.
 *  @param range The measured distances.
 *  @param forward_offset Offset of the laser in x direction.
 *  @param angular_resolution The angle between to beams.
 *  @param laser_maxrange The angle between to beams.
 *  @param first_beam_angle Angle of the first beam of a laser (often -0.5*PI)
 *  @param backwards Is it the rearlaser (=1) or the frontlaser(=0)
**/
void 
localize_incorporate_laser(localize_particle_filter_p filter,
                           localize_map_p map,
                           int num_readings,
                           float *range,
                           double forward_offset,
                           double angular_resolution,
                           double laser_maxrange,
                           double first_beam_angle,
                           int backwards);


/** Carries out the resampling step.
 *
 *  @param filter Particle filter structure the function is applied to.
 **/
void 
localize_resample(localize_particle_filter_p filter);

/** Carry out the three steps of the paricle filter which are:
 *  1) Draw from the motion model. 2) Compute the importance weights. 3) Resample.
 *
 *  @param filter Particle filter structure the function is applied to.
 *  @param map Map of the environment.
 *  @param laser A robot_laser_message which incorporates odoemtry, laser, and its configuration
 *  @param forward_offset Offset of the laser in x direction.
 *  @param backwards Is it the rearlaser (=1) or the frontlaser(=0)
 **/
void 
localize_run(localize_particle_filter_p filter,
             localize_map_p map,
             robot_laser_message *laser,
             double forward_offset,
             int backwards);

void 
localize_laser_scan_gd(int num_readings,
                       float *range,
                       double angular_resolution,
                       double first_beam_angle,
                       point_p laser_pos,
                       double forward_offset,
                       localize_map_p map,
                       int laser_skip);

/** Carry a summary for sending it via ipc to other modules liek the gui **/
void 
localize_summarize(localize_particle_filter_p filter,
                   localize_summary_p summary,
                   localize_map_p map,
                   int num_readings,
                   float *range,
                   double forward_offset,
                   double angular_resolution,
                   double first_beam_angle,
                   int backwards);

//add from ros amcl
static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}
double pf_ran_gaussian(double sigma);

#ifdef __cplusplus
}
#endif

#endif
// @}
