
#include "localizecore.h"
#include "localize_motion.h"

/* gains for gradient descent */

#define K_T   0.0001
#define K_ROT 0.00001

/* priority queue for sorting global localization hypotheses */

typedef struct queue_node {
    point_t point;
    float prob;
    struct queue_node *next, *prev;
} queue_node_t, *queue_node_p;

typedef struct {
    int num_elements, max_elements;
    queue_node_p first, last;
} priority_queue_t, *priority_queue_p;

/* initialize a new priority queue */

static priority_queue_p priority_queue_init(int max_elements)
{
    priority_queue_p result;

    result = (priority_queue_p)calloc(1, sizeof(priority_queue_t));
    //test_alloc(result);
    result->num_elements = 0;
    result->max_elements = max_elements;
    result->first = NULL;
    result->last = NULL;
    return result;
}

/* add a point to the priority queue */

static void priority_queue_add(priority_queue_p queue, point_t point,
                               float prob)
{
    queue_node_p mark, temp;

    if(queue->num_elements == 0) {
        temp = (queue_node_p)calloc(1, sizeof(queue_node_t));
        //test_alloc(temp);
        temp->point = point;
        temp->prob = prob;
        temp->prev = NULL;
        temp->next = NULL;
        queue->first = temp;
        queue->last = temp;
        queue->num_elements++;
    }
    else if(prob > queue->last->prob ||
            queue->num_elements < queue->max_elements) {
        mark = queue->last;
        while(mark != NULL && prob > mark->prob)
            mark = mark->prev;
        if(mark == NULL) {
            temp = (queue_node_p)calloc(1, sizeof(queue_node_t));
            //test_alloc(temp);
            temp->point = point;
            temp->prob = prob;
            temp->prev = NULL;
            temp->next = queue->first;
            queue->first->prev = temp;
            queue->first = temp;
            queue->num_elements++;
        }
        else {
            temp = (queue_node_p)calloc(1, sizeof(queue_node_t));
            //test_alloc(temp);
            temp->point = point;
            temp->prob = prob;
            temp->prev = mark;
            temp->next = mark->next;
            if(mark->next != NULL)
                mark->next->prev = temp;
            else
                queue->last = temp;
            mark->next = temp;
            queue->num_elements++;
        }
        if(queue->num_elements > queue->max_elements) {
            queue->last = queue->last->prev;
            free(queue->last->next);
            queue->last->next = NULL;
            queue->num_elements--;
        }
    }
}

/* free the priority queue */

static void priority_queue_free(priority_queue_p queue)
{
    queue_node_p mark;

    while(queue->first != NULL) {
        mark = queue->first;
        queue->first = queue->first->next;
        free(mark);
    }
    free(queue);
}

/* initialize memory necessary for holding temporary sensor weights */

static void initialize_temp_weights(localize_particle_filter_p filter)
{
    int i;

    filter->temp_weights = (float **)calloc(filter->param->num_particles,
                                            sizeof(float *));
    //test_alloc(filter->temp_weights);
    for(i = 0; i < filter->param->num_particles; i++) {
        filter->temp_weights[i] = (float *)calloc(MAX_BEAMS_PER_SCAN, sizeof(float));
        //test_alloc(filter->temp_weights[i]);
    }
}

/* resize memory for temporary sensor weights */

static void realloc_temp_weights(localize_particle_filter_p filter,
                                 int num_particles)
{
    int i;

    for(i = 0; i < filter->param->num_particles; i++)
        free(filter->temp_weights[i]);
    filter->temp_weights = (float **)realloc(filter->temp_weights,
                                             num_particles * sizeof(float *));
    //test_alloc(filter->temp_weights);
    for(i = 0; i < num_particles; i++) {
        filter->temp_weights[i] = (float *)calloc(MAX_BEAMS_PER_SCAN, sizeof(float));
        //test_alloc(filter->temp_weights[i]);
    }
}

/* allocate memory for a new particle filter */

localize_particle_filter_p
localize_particle_filter_new(localize_param_p param)
{
    localize_particle_filter_p filter;

    /* allocate the particle filter */
    filter =
            (localize_particle_filter_p)
            calloc(1, sizeof(localize_particle_filter_t));
    //test_alloc(filter);

    /* set the parameters */
    filter->param = param;

    /* allocate the initial particle set */
    filter->particles =
            (localize_particle_p)calloc(filter->param->num_particles,
                                        sizeof(localize_particle_t));
    //test_alloc(filter->particles);

    /* initialize the temporary weights */
    initialize_temp_weights(filter);

    /* filter has not been initialized */
    filter->initialized = 0;
    filter->first_odometry = 1;
    filter->global_mode = 0;
    filter->distance_travelled = 0;
    filter->angle_rotated=0.0;

    filter->param->laser_skip = 0; /* will be automatically initialized later on */

    return filter;
}

void localize_initialize_particles_uniform(localize_particle_filter_p filter,
                                           robot_laser_message *laser,
                                           localize_map_p map)
{
    priority_queue_p queue = priority_queue_init(filter->param->num_particles);
    float *laser_x, *laser_y;
    int i, j, x_l, y_l;
    float angle, prob, ctheta, stheta;
    point_t point;
    queue_node_p mark;
    int *beam_valid;


    /* compute the correct laser_skip */
    if (filter->param->laser_skip <= 0) {
        filter->param->laser_skip =
                floor(filter->param->integrate_angle / laser->config.angular_resolution);
    }

    fprintf(stderr, "\rDoing global localization... (%.1f%% complete)", 0.0);
    filter->initialized = 0;
    /* copy laser scan into temporary memory */
    laser_x = (float *)calloc(laser->num_readings, sizeof(float));
    //test_alloc(laser_x);
    laser_y = (float *)calloc(laser->num_readings, sizeof(float));
    //test_alloc(laser_y);
    beam_valid = (int *)calloc(laser->num_readings, sizeof(int));
    //test_alloc(beam_valid);

    for(i = 0; i < laser->num_readings; i++) {
        if (laser->range[i] < laser->config.maximum_range &&
                laser->range[i] < filter->param->max_range)
            beam_valid[i] = 1;
        else
            beam_valid[i] = 0;
    }

    /* do all calculations in map coordinates */
    for(i = 0; i < laser->num_readings; i++) {
		//crti:原来的,2016-03-21
        //angle = laser->config.start_angle +
        //        i * laser->config.angular_resolution;

        //laser_x[i] = (filter->param->front_laser_offset +
        //              laser->range[i] * cos(angle)) / map->config.resolution;
        //laser_y[i] = (0 +
        //              laser->range[i] * sin(angle)) / map->config.resolution;

		angle = laser->config.start_angle +
                i * laser->config.angular_resolution;

        laser_x[i] = (0 +
                      laser->range[i] * cos(angle)) / map->config.resolution;
        laser_y[i] = (0 +
                      laser->range[i] * sin(angle)) / map->config.resolution;
    }

    for(i = 0; i < filter->param->global_test_samples; i++) {
        if(i % 10000 == 0) {
            fprintf(stderr, "\rDoing global localization... (%.1f%% complete)",
                    i / (float)filter->param->global_test_samples * 100.0);
            //ipc_sleep(0.001);
        }
        do {
            point.x = uniform_random(0, map->config.x_size - 1);
            point.y = uniform_random(0, map->config.y_size - 1);
        } while(map->map.map[(int)point.x][(int)point.y] >
                filter->param->occupied_prob ||
                map->map.map[(int)point.x][(int)point.y] == -1);
        point.theta = uniform_random(-M_PI, M_PI);

        prob = 0.0;
        ctheta = cos(point.theta);
        stheta = sin(point.theta);
        for(j = 0; j < laser->num_readings &&
            (queue->last == NULL || prob > queue->last->prob);
            j += filter->param->laser_skip) {

            if (beam_valid[j]) {
                x_l = point.x + laser_x[j] * ctheta - laser_y[j] * stheta;
                y_l = point.y + laser_x[j] * stheta + laser_y[j] * ctheta;

                if(x_l >= 0 && y_l >= 0 && x_l < map->config.x_size &&
                        y_l < map->config.y_size)
                    prob += map->gprob[x_l][y_l];
                else
                    prob -= 100;
            }
        }
        priority_queue_add(queue, point, prob);
    }

    /* transfer samples from priority queue back into particles */
    mark = queue->first;
    for(i = 0; i < queue->num_elements; i++) {
        filter->particles[i].x = mark->point.x * map->config.resolution;
        filter->particles[i].y = mark->point.y * map->config.resolution;
        filter->particles[i].theta = mark->point.theta;
        mark = mark->next;
    }
    priority_queue_free(queue);
    free(laser_x);
    free(laser_y);
    free(beam_valid);


    if(filter->param->do_scanmatching) {
        for(i = 0; i < filter->param->num_particles; i++) {
            point.x = filter->particles[i].x;
            point.y = filter->particles[i].y;
            point.theta = filter->particles[i].theta;
            localize_laser_scan_gd(laser->num_readings, laser->range,
                                   laser->config.angular_resolution,
                                   laser->config.start_angle,
                                   &point,
                                   filter->param->front_laser_offset,
                                   map,
                                   filter->param->laser_skip);
            filter->particles[i].x = point.x;
            filter->particles[i].y = point.y;
            filter->particles[i].theta = point.theta;
            filter->particles[i].weight = 0.0;
        }
    }
    filter->initialized = 1;
    filter->first_odometry = 1;
    filter->global_mode = 1;
    filter->distance_travelled = 0;
    filter->angle_rotated=0.0;
    fprintf(stderr, "\rDoing global localization... (%.1f%% complete)\n\n",
            100.0);
}

/* initialize particles from a gaussian distribution */

void localize_initialize_particles_gaussians(localize_particle_filter_p filter,
                                             int num_modes,
                                             point_t *mean,
                                             point_t *std)
{
    int i, j, each, start, end;
    float x, y, theta;

    each = (int)floor(filter->param->num_particles / (float)num_modes);
    for(i = 0; i < num_modes; i++) {
        start = i * each;
        if(i == num_modes - 1)
            end = filter->param->num_particles;
        else
            end = (i + 1) * each;

        for(j = start; j < end; j++) {
            x = gaussian_random(mean[i].x, std[i].x);
            y = gaussian_random(mean[i].y, std[i].y);
            theta = normalize_theta(gaussian_random(mean[i].theta,
                                                    std[i].theta));
            filter->particles[j].x = x;
            filter->particles[j].y = y;
            filter->particles[j].theta = theta;
            filter->particles[j].weight = 0.0;
        }
    }

    filter->initialized = 1;
    filter->first_odometry = 1;
    if (num_modes < 2)
        filter->global_mode = 0;
    else
        filter->global_mode = 1;
    filter->distance_travelled = 0;
    filter->angle_rotated=0.0;
}

int localize_initialize_particles_placename(localize_particle_filter_p filter,
                                            map_placelist_p placelist,
                                            char *placename)
{
    point_t mean, std;

    int i;
    for(i = 0; i < placelist->num_places; i++)
        if(strcmp(placename, placelist->places[i].name) == 0)
            break;
    /*   if(i == placelist->num_places ||  */
    /*      placelist->places[i].type != CARMEN_LCALIZATION_INIT_TYPE) */
    /*     return -1; */
    if(i == placelist->num_places/*  ||  */
            /*      placelist->places[i].type != CARMEN_LCALIZATION_INIT_TYPE */)
        return -1;
    mean.x = placelist->places[i].x;
    mean.y = placelist->places[i].y;
    mean.theta = placelist->places[i].theta;
    std.x = placelist->places[i].x_std;
    std.y = placelist->places[i].y_std;
    std.theta = placelist->places[i].theta_std;
    localize_initialize_particles_gaussian(filter, mean, std);
    return 0;
}

/* initialize particles from a gaussian distribution */

void localize_initialize_particles_gaussian(localize_particle_filter_p filter,
                                            point_t mean,
                                            point_t std)
{
    localize_initialize_particles_gaussians(filter, 1, &mean, &std);
}

/* initialize particle positions and weights from parameters */

void localize_initialize_particles_manual(localize_particle_filter_p filter,
                                          double *x, double *y, double *theta,
                                          double *weight, int num_particles)
{
    int i;

    if(num_particles != filter->param->num_particles) {
        filter->particles =
                (localize_particle_p)realloc(filter->particles, num_particles *
                                             sizeof(localize_particle_t));
        //test_alloc(filter->particles);
        realloc_temp_weights(filter, num_particles);
        filter->param->num_particles = num_particles;
    }
    for(i = 0; i < filter->param->num_particles; i++) {
        filter->particles[i].x = x[i];
        filter->particles[i].y = y[i];
        filter->particles[i].theta = theta[i];
        filter->particles[i].weight = weight[i];
    }
    filter->initialized = 1;
    filter->first_odometry = 1;
    filter->global_mode = 0;
    filter->distance_travelled = 0;
    filter->angle_rotated=0.0;
}

double pf_ran_gaussian(double sigma)
{
  double x1, x2, w, r;

  do
  {
    do { r = ((double)rand())/RAND_MAX; } while (r==0.0);
    x1 = 2.0 * r - 1.0;
    do { r = ((double)rand())/RAND_MAX; } while (r==0.0);
    x2 = 2.0 * r - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);

  return(sigma * x2 * sqrt(-2.0*log(w)/w));
}


void localize_incorporate_odometry_diff(localize_particle_filter_p filter,
                                   point_t odometry_position)
{
    int i, backwards;
    double delta_t, delta_theta;
    double dx, dy, odom_theta;
    double downrange, crossrange, turn;

    /* The dr1/dr2 code becomes unstable if dt is too small. */
    if(filter->first_odometry) {
        filter->last_odometry_position = odometry_position;
        filter->first_odometry = 0;
        return;
    }

    dx = odometry_position.x - filter->last_odometry_position.x;
    dy = odometry_position.y - filter->last_odometry_position.y;
    delta_t = sqrt(dx * dx + dy * dy);
    delta_theta = normalize_theta(odometry_position.theta -
                                  filter->last_odometry_position.theta);
    odom_theta = atan2(dy, dx);
    backwards = (dx * cos(odometry_position.theta) +
                 dy * sin(odometry_position.theta) < 0);

    filter->distance_travelled += delta_t;
    filter->angle_rotated += fabs(delta_theta);

    for(i = 0; i < filter->param->num_particles; i++){
        downrange =
                localize_sample_noisy_downrange(delta_t, delta_theta,
                                                filter->param->motion_model);
        crossrange =
                localize_sample_noisy_crossrange(delta_t, delta_theta,
                                                 filter->param->motion_model);
        turn = localize_sample_noisy_turn(delta_t, delta_theta,
                                          filter->param->motion_model);

        if(backwards) {
            filter->particles[i].x -= downrange *
                    cos(filter->particles[i].theta + turn/2.0) +
                    crossrange * cos(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
            filter->particles[i].y -= downrange *
                    sin(filter->particles[i].theta + turn/2.0) +
                    crossrange * sin(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
        } else {
            filter->particles[i].x += downrange *
                    cos(filter->particles[i].theta + turn/2.0) +
                    crossrange * cos(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
            filter->particles[i].y += downrange *
                    sin(filter->particles[i].theta + turn/2.0) +
                    crossrange * sin(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
        }
        filter->particles[i].theta = normalize_theta(filter->particles[i].theta+turn);
    }
    /* keep track of the last odometry */
    filter->last_odometry_position = odometry_position;
}

void localize_incorporate_odometry_omni(localize_particle_filter_p filter,
                                   point_t odometry_position)
{
    int i, backwards,rightwards;
    double delta_t, delta_theta;
    double dx, dy, odom_theta;
    double downrange, crossrange, turn;

    if(filter->first_odometry) {
        filter->last_odometry_position = odometry_position;
        filter->first_odometry = 0;
        return;
    } 

    dx = odometry_position.x - filter->last_odometry_position.x;
    dy = odometry_position.y - filter->last_odometry_position.y;
    delta_t = sqrt(dx * dx + dy * dy);
    delta_theta = normalize_theta(odometry_position.theta -
                                  filter->last_odometry_position.theta);
    odom_theta = atan2(dy, dx);
    backwards = (dx * cos(odometry_position.theta) +
                 dy * sin(odometry_position.theta) < 0);
	rightwards = (- dx * sin(odometry_position.theta) +
                 dy * cos(odometry_position.theta) < 0);

    filter->distance_travelled += delta_t;
    filter->angle_rotated+=fabs(delta_theta);

    for(i = 0; i < filter->param->num_particles; i++) {
        downrange =
                localize_sample_noisy_downrange(delta_t, delta_theta,
                                                filter->param->motion_model);
        crossrange =
                localize_sample_noisy_crossrange(delta_t, delta_theta,
                                                 filter->param->motion_model);
        turn = localize_sample_noisy_turn(delta_t, delta_theta,
                                          filter->param->motion_model);

		if(backwards)
		{
			downrange = -downrange;
		}
		if(rightwards)
		{
			crossrange = -crossrange;
		}
		//crti:2016-06-21,原来没有考虑左右，只有backwards没有rightwards，要在diff平台上测试一下
        filter->particles[i].x += downrange *
                cos(filter->particles[i].theta + turn/2.0) +
                crossrange * cos(filter->particles[i].theta + turn/2.0 + M_PI/2.0);
        filter->particles[i].y += downrange *
                sin(filter->particles[i].theta + turn/2.0) +
                crossrange * sin(filter->particles[i].theta + turn/2.0 + M_PI/2.0);

        filter->particles[i].theta = normalize_theta(filter->particles[i].theta+turn);
    }
    /* keep track of the last odometry */
    filter->last_odometry_position = odometry_position;
}

/*
检测是否存在某个粒子，其与中心的偏离程度过高
如果是就定义为所谓的global_mode
*/
static int global_mode_test(localize_particle_filter_p filter)
{
    int i;
    float mean_x = 0, mean_y = 0;

    for(i = 0; i < filter->param->num_particles; i++) {
        mean_x += filter->particles[i].x;
        mean_y += filter->particles[i].y;
    }
    mean_x /= filter->param->num_particles;
    mean_y /= filter->param->num_particles;

    for(i = 0; i < filter->param->num_particles; i++)
        if(fabs(filter->particles[i].x - mean_x) >
                filter->param->global_distance_threshold ||
                fabs(filter->particles[i].y - mean_y) >
                filter->param->global_distance_threshold)
            return 1;
    return 0;
}

/* incorporate a single laser scan into the paritcle filter */
void localize_incorporate_laser(localize_particle_filter_p filter,
                                localize_map_p map, int num_readings,
                                float *range, double forward_offset,
                                double angular_resolution,
                                double laser_maxrange,
                                double first_beam_angle,
                                int backwards)
{
    float angle, *laser_x, *laser_y, p_x, p_y, ctheta, stheta;

    float log_small_prob = log(filter->param->tracking_beam_minlikelihood);
    float global_log_small_prob = log(filter->param->global_beam_minlikelihood);
    float log_min_wall_prob = log(filter->param->tracking_beam_minlikelihood);

    int i, j, x, y, robot_x, robot_y;
    int count[4096];

    /* compute the correct laser_skip */
    if (filter->param->laser_skip <= 0) {
        filter->param->laser_skip =
                floor(filter->param->integrate_angle / angular_resolution);
    }

    assert(filter->param->laser_skip);

    /* reset the weights back to even */
    for(i = 0; i < filter->param->num_particles; i++)
        filter->particles[i].weight = 0.0;

    /* compute positions of laser points assuming robot pos is (0, 0, 0) */
    laser_x = (float *)calloc(num_readings, sizeof(float));
    laser_y = (float *)calloc(num_readings, sizeof(float));

    for(i = 0; i < num_readings; i++) {
        angle = first_beam_angle + i * angular_resolution;
		laser_x[i] = (0 + range[i] * cos(angle)) / map->config.resolution;
        laser_y[i] = (range[i] * sin(angle)) / map->config.resolution;
        if(backwards) {
            laser_x[i] = -laser_x[i];
            laser_y[i] = -laser_y[i];
        }
        if((i % filter->param->laser_skip) == 0 &&
                range[i] < filter->param->max_range &&
                range[i] < laser_maxrange)
            filter->laser_mask[i] = 1;
        else
            filter->laser_mask[i] = 0;
    }

    /* test for global mode */
    filter->global_mode = global_mode_test(filter);

    if(filter->global_mode)
        /* compute weight of each laser reading - using global map */
        for(i = 0; i < filter->param->num_particles; i++) {
            p_x = filter->particles[i].x / map->config.resolution;
            p_y = filter->particles[i].y / map->config.resolution;
            ctheta = cos(filter->particles[i].theta);
            stheta = sin(filter->particles[i].theta);
            for(j = 0; j < num_readings; j += filter->param->laser_skip)
                if(filter->laser_mask[j]) {
                    x = (p_x + laser_x[j] * ctheta - laser_y[j] * stheta);
                    y = (p_y + laser_x[j] * stheta + laser_y[j] * ctheta);
                    robot_x = p_x;
                    robot_y = p_y;
                    if(x < 0 || y < 0 || x >= map->config.x_size ||
                            y >= map->config.y_size || map->map.map[x][y] == -1)
                        filter->particles[i].weight += global_log_small_prob;
                    else if(filter->param->constrain_to_map &&
                            (robot_x < 0 || robot_y < 0 ||
                             robot_x >= map->config.x_size ||
                             robot_y >= map->config.y_size ||
                             map->map.map[robot_x][robot_y] >
                             filter->param->occupied_prob))
                        filter->particles[i].weight += global_log_small_prob;
                    else
                        filter->particles[i].weight += map->gprob[x][y];                }
        }
    else {
        /* compute weight of each laser reading */
        for(i = 0; i < filter->param->num_particles; i++) {
            p_x = filter->particles[i].x / map->config.resolution;
            p_y = filter->particles[i].y / map->config.resolution;
            ctheta = cos(filter->particles[i].theta);
            stheta = sin(filter->particles[i].theta);
            for(j = 0; j < num_readings; j += filter->param->laser_skip)
                if(filter->laser_mask[j]) {
                    x = (p_x + laser_x[j] * ctheta - laser_y[j] * stheta);
                    y = (p_y + laser_x[j] * stheta + laser_y[j] * ctheta);
                    robot_x = p_x;
                    robot_y = p_y;
                    if(x < 0 || y < 0 || x >= map->config.x_size ||
                            y >= map->config.y_size || map->map.map[x][y] == -1)
                        filter->temp_weights[i][j] = log_small_prob;
                    else if(filter->param->constrain_to_map &&
                            (robot_x < 0 || robot_y < 0 ||
                             robot_x >= map->config.x_size ||
                             robot_y >= map->config.y_size ||
                             map->map.map[robot_x][robot_y] >
                             filter->param->occupied_prob))
                        filter->temp_weights[i][j] = log_small_prob;
                    else
                        filter->temp_weights[i][j] = map->prob[x][y];
                }
        }

        /* ignore laser readings that are improbable in a large fraction of the particles */
        memset(count, 0, num_readings * sizeof(int));
        for(i = 0; i < filter->param->num_particles; i++)
            for(j = 0; j < num_readings; j += filter->param->laser_skip)
                if(filter->laser_mask[j] && filter->temp_weights[i][j] < log_min_wall_prob)
                    count[j]++;

        for(i = 0; i < num_readings; i++)
            if(filter->laser_mask[i] && count[i] / (float)filter->param->num_particles > filter->param->outlier_fraction)
                filter->laser_mask[i] = 0;

        /* add log probabilities to particle weights */
        for(i = 0; i < filter->param->num_particles; i++)
            for(j = 0; j < num_readings; j += filter->param->laser_skip)
                if(filter->laser_mask[j])
					filter->particles[i].weight += filter->temp_weights[i][j];
    }

    /* free laser points */
    free(laser_x);
    free(laser_y);
}

/* resample particle filter */

void localize_resample(localize_particle_filter_p filter)
{
    int i, which_particle;
    float weight_sum = 0.0, *cumulative_sum = NULL;
    float position, step_size, max_weight = filter->particles[0].weight;
    localize_particle_p temp_particles = NULL;

    /* change log weights back into probabilities */
    for(i = 0; i < filter->param->num_particles; i++)
        if(filter->particles[i].weight > max_weight)
            max_weight = filter->particles[i].weight;
    for(i = 0; i < filter->param->num_particles; i++)
        filter->particles[i].weight =
                exp(filter->particles[i].weight - max_weight);

    /* Allocate memory necessary for resampling */
    cumulative_sum = (float *)calloc(filter->param->num_particles, sizeof(float));
    //test_alloc(cumulative_sum);记录的是权重
    temp_particles = (localize_particle_p)
            calloc(filter->param->num_particles, sizeof(localize_particle_t));
    //test_alloc(temp_particles);记录的是粒子

    /* Sum the weights of all of the particles */
    for(i = 0; i < filter->param->num_particles; i++) {
        weight_sum += filter->particles[i].weight;
        cumulative_sum[i] = weight_sum;
    }

    /* choose random starting position for low-variance walk */
    position = uniform_random(0, weight_sum);
	//printf("%f",position);
	//std::cout << "position from" << position << std::endl;
    step_size = weight_sum / (float)filter->param->num_particles;
    which_particle = 0;

    /* draw num_particles random samples */
    for(i = 0; i < filter->param->num_particles; i++) {
        position += step_size;
        if(position > weight_sum) {
            position -= weight_sum;
            which_particle = 0;
        }
        while(position > cumulative_sum[which_particle])
            which_particle++;
        memcpy(temp_particles + i, filter->particles + which_particle,
               sizeof(localize_particle_t));
    }

    /* Copy new particles back into the filter. */
    free(filter->particles);
    filter->particles = temp_particles;
    free(cumulative_sum);

    /* set all log weights back to zero */
    for(i = 0; i < filter->param->num_particles; i++)
        filter->particles[i].weight = 0.0;
}

/* incorporate a robot laser reading */
void localize_run(localize_particle_filter_p filter, localize_map_p map,
                  robot_laser_message *laser, double forward_offset,
                  int backwards)
{
    point_t robot_position;

    if(!filter->initialized)
        return;
    /* incorporate the laser position stamp */
	robot_position.x = laser->laser_pose.x;
    robot_position.y = laser->laser_pose.y;
    robot_position.theta = laser->laser_pose.theta;

	//crti:2016-07-17,添加diff和omni的两种里程计输入，原来是diff
    localize_incorporate_odometry_diff(filter, robot_position);

    if(filter->param->use_sensor) {
        /* incorporate the laser scan */
		// 加入pdf修正，读入静态
        localize_incorporate_laser(filter, 
								   map,
								   laser->num_readings,
                                   laser->range, 
								   forward_offset,
                                   laser->config.angular_resolution,
                                   laser->config.maximum_range,
                                   laser->config.start_angle,
                                   backwards);

        /* check if it is time to resample */
        if ( (filter->distance_travelled > filter->param->update_distance)||(filter->angle_rotated>filter->param->update_angle))
        {
            localize_resample(filter);
            filter->distance_travelled = 0;
            filter->angle_rotated=0.0;
            filter->initialized = 1;
        }
    }
}

void localize_laser_scan_gd(int num_readings, float *range,
                            double angular_resolution,
                            double first_beam_angle,
                            point_p laser_pos, double forward_offset,
                            localize_map_p map, int laser_skip)
{
    float grad_x, grad_y, grad_theta, range_x, range_y, theta;
    int x_l, y_l, count = 0, i;

    double angular_res_in_degrees = radians_to_degrees(angular_resolution);


    do {
        grad_x = 0;
        grad_y = 0;
        grad_theta = 0;
        for(i = 0; i < num_readings; i += laser_skip) {

            theta = laser_pos->theta + first_beam_angle + i * angular_resolution;

            range_x = range[i] * cos(theta);
            range_y = range[i] * sin(theta);
            x_l = (int)((laser_pos->x + forward_offset * cos(laser_pos->theta) +
                         range_x) / map->config.resolution);
            y_l = (int)((laser_pos->y + forward_offset * sin(laser_pos->theta) +
                         range_y) / map->config.resolution);

            if(x_l >= 0 && y_l >= 0 && x_l < map->config.x_size &&
                    y_l < map->config.y_size) {
                grad_x += map->x_offset[x_l][y_l];
                grad_y += map->y_offset[x_l][y_l];
                grad_theta += range_x * map->y_offset[x_l][y_l] -
                        range_y * map->x_offset[x_l][y_l];
            }
        }

        /** what is the meaning of this? should this be adapted according to the fov ?*/
        /*     grad_x *= K_T * 180.0 / num_readings; */
        /*     grad_y *= K_T * 180.0 / num_readings; */
        /*     grad_theta *= K_ROT * 180.0 / num_readings; */

        grad_x *= K_T * angular_res_in_degrees;
        grad_y *= K_T * angular_res_in_degrees;
        grad_theta *= K_ROT * angular_res_in_degrees;

        laser_pos->x += grad_x;
        laser_pos->y += grad_y;
        laser_pos->theta += grad_theta;
        count++;
    } while(count < 20 && (grad_x > 0.05 || grad_y < 0.05 ||
                           grad_theta < 0.25 * 180.0 / M_PI));
}

void localize_summarize(localize_particle_filter_p filter,
                        localize_summary_p summary,
                        localize_map_p map,
                        int num_readings, float *range,
                        double angular_resolution,
                        double first_beam_angle,
                        double forward_offset,
                        int backwards)
{
    float mean_x, mean_y, mean_theta_x, mean_theta_y, angle;
    float diff_x, diff_y, diff_theta, std_x, std_y, std_theta, xy_cov;
    float *weights, max_weight = filter->particles[0].weight;
    float total_weight = 0;
    int i, x, y;

    summary->converged = !filter->global_mode;

    weights = (float *)calloc(filter->param->num_particles, sizeof(float));
    //test_alloc(weights);
    for(i = 0; i < filter->param->num_particles; i++)
        if(filter->particles[i].weight > max_weight)
            max_weight = filter->particles[i].weight;
    for(i = 0; i < filter->param->num_particles; i++) {
        weights[i] = exp(filter->particles[i].weight - max_weight);
        total_weight += weights[i];
    }

	float mat;
	mat = total_weight / filter->param->num_particles;
	summary->match = mat;
	//printf("match = %d",summary->match);
	//fprintf(stderr, "match = %d", summary->match);
    /* compute mean particle pose */
    mean_x = 0;
    mean_y = 0;
    mean_theta_x = 0;
    mean_theta_y = 0;
    for(i = 0; i < filter->param->num_particles; i++) {
        mean_x += filter->particles[i].x * weights[i];
        mean_y += filter->particles[i].y * weights[i];
        mean_theta_x += cos(filter->particles[i].theta) * weights[i];
        mean_theta_y += sin(filter->particles[i].theta) * weights[i];
    }
    summary->mean.x = mean_x / total_weight;
    summary->mean.y = mean_y / total_weight;
    if(mean_theta_x == 0)
        summary->mean.theta = 0;
    else
        summary->mean.theta = atan2(mean_theta_y, mean_theta_x);
    summary->odometry_pos = filter->last_odometry_position;

    /* compute std particle pose */
    std_x = 0;
    std_y = 0;
    std_theta = 0;
    xy_cov = 0;
    for(i = 0; i < filter->param->num_particles; i++) {
        diff_x = (filter->particles[i].x - summary->mean.x);
        diff_y = (filter->particles[i].y - summary->mean.y);
        diff_theta = normalize_theta(filter->particles[i].theta -
                                     summary->mean.theta);
        std_x += square(diff_x);
        std_y += square(diff_y);
        std_theta += square(diff_theta);
        xy_cov += diff_x * diff_y;
    }
    summary->std.x = sqrt(std_x / filter->param->num_particles);
    summary->std.y = sqrt(std_y / filter->param->num_particles);
    summary->std.theta = sqrt(std_theta / filter->param->num_particles);
    summary->xy_cov = sqrt(xy_cov / filter->param->num_particles);

    if(filter->param->do_scanmatching)
        localize_laser_scan_gd(summary->num_readings,
                               range,
                               angular_resolution,
                               first_beam_angle,
                               &summary->mean,
                               forward_offset, map, 1);

    /* compute mean scan */	
    summary->num_readings = num_readings;
	summary->mean_scan[num_readings].prob = 0;
	int cnt = 0 ;
	for (i = 0; i < num_readings; i++) {
		summary->mean_scan[i].range = range[i];
		summary->mean_scan[i].mask = filter->laser_mask[i];
		
		if (backwards) {
			angle = summary->mean.theta + M_PI +
				first_beam_angle + i * angular_resolution;
			summary->mean_scan[i].x = summary->mean.x - forward_offset *
				cos(summary->mean.theta) + cos(angle) * range[i];
			summary->mean_scan[i].y = summary->mean.y - forward_offset *
				sin(summary->mean.theta) + sin(angle) * range[i];
		}
		else {
			angle = summary->mean.theta +
				first_beam_angle + i * angular_resolution;
			summary->mean_scan[i].x = summary->mean.x + forward_offset *
				cos(summary->mean.theta) + cos(angle) * range[i];
			summary->mean_scan[i].y = summary->mean.y + forward_offset *
				sin(summary->mean.theta) + sin(angle) * range[i];
		}

		x = (summary->mean_scan[i].x / map->config.resolution);
		y = (summary->mean_scan[i].y / map->config.resolution);

		if (x < 0 || y < 0 || x >= map->config.x_size || y >= map->config.y_size || map->map.map[x][y] == -1)
			summary->mean_scan[i].prob = 0; //SMALL_PROB;
		else
		{
			summary->mean_scan[i].prob = exp(map->prob[x][y]);
			cnt += 1;
		}
		//printf("now the prob[%d]=%f\n", i, summary->mean_scan[i].prob);
		if (summary->mean_scan[i].prob >= 0.5){

			summary->mean_scan[num_readings].prob += 1;
		}
	}
	printf("total points in map: %d\n", cnt);
    free(weights);
}


