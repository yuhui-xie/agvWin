
#include "map_modify.h"

#define LASER_HISTORY_LENGTH 5

#define EMPTY 0.01
#define FILLED .9
#define UNKNOWN -1

typedef struct {
    int x, y;
    double value;
} grid_cell_t, *grid_cell_p;

static grid_cell_p *laser_scan;
static int *scan_size;
static int *max_scan_size;
static int current_data_set = 0;
inline double normalize_theta(double theta)
{
    int multiplier;

    if (theta >= -M_PI && theta < M_PI)
        return theta;

    multiplier = (int)(theta / (2*M_PI));
    theta = theta - multiplier*2*M_PI;
    if (theta >= M_PI)
        theta -= 2*M_PI;
    if (theta < -M_PI)
        theta += 2*M_PI;

    return theta;
}

void
get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

  if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

inline double distance(point_p p1, point_p p2)
{
  return sqrt((p1->x-p2->x)*(p1->x-p2->x) + (p1->y-p2->y)*(p1->y-p2->y));
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y,
                   bresenham_param_t *params);
inline void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
    *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
    *y = -*y;
    }
}

inline int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}
inline static int
is_empty(double value) 
{
    return (value >= 0) && (value <= EMPTY);
}

inline static int
is_filled(double value) 
{
    return (value >= 0) && (value >= FILLED);
}

inline static int
is_in_map(int x, int y, map_p map)
{
    if (x < 0 || x >= map->config.x_size)
        return 0;
    if (y < 0 || y >= map->config.y_size)
        return 0;
    return 1;
}

static int 
point_exists(int x, int y, double value) 
{
    int num_points = scan_size[current_data_set];
    grid_cell_p cell = laser_scan[current_data_set];
    int index;

    if (num_points == 0)
        return 0;

    for (index = num_points-1; index >= 0; index--) {
        if (x == cell[index].x && y == cell[index].y &&
                fabs(value - cell[index].value) < .01)
            return 1;
    }

    return 0;
}

static void 
add_filled_point(int x, int y, map_p true_map, map_p modify_map)
{
    int num_points;
    grid_cell_p cell_list;
    double value;

    if (!is_in_map(x, y, modify_map))
        return;

    value = true_map->map[x][y];
    if (is_filled(value))
        return;

    /*   if (point_exists(x, y, FILLED)) */
    /*     return; */

    num_points = scan_size[current_data_set];
    if (num_points == max_scan_size[current_data_set]) {
        max_scan_size[current_data_set] *= 2;
        laser_scan[current_data_set] =
                realloc(laser_scan[current_data_set],
                        max_scan_size[current_data_set]*sizeof(grid_cell_t));
        assert(laser_scan[current_data_set]);
    }

    cell_list = laser_scan[current_data_set];
    value = FILLED;
    /*      warn("Filling point %d (%d) %d %d\n", num_points, */
    /*  	 current_data_set, x, y); */
    cell_list[num_points].x = x;
    cell_list[num_points].y = y;
    cell_list[num_points].value = value;
    scan_size[current_data_set]++;
    modify_map->map[x][y] = 2.0;
}

static void 
add_clear_point(int x, int y, map_p true_map, map_p modify_map)
{
    int num_points = scan_size[current_data_set];
    grid_cell_p cell_list;
    double value;

    if (!is_in_map(x, y, modify_map))
        return;

    value = true_map->map[x][y];

    /*   if (is_empty(value))  */
    /* 		return; */

    if (point_exists(x, y, EMPTY))
        return;

    if (num_points == max_scan_size[current_data_set]) {
        max_scan_size[current_data_set] *= 2;
        laser_scan[current_data_set] =
                realloc(laser_scan[current_data_set],
                        max_scan_size[current_data_set]*sizeof(grid_cell_t));
        assert(laser_scan[current_data_set]);
    }

    cell_list = laser_scan[current_data_set];
    value = EMPTY;
    cell_list[num_points].x = x;
    cell_list[num_points].y = y;
    cell_list[num_points].value = value;
    scan_size[current_data_set]++;
    modify_map->map[x][y] = value;
}

static void 
update_existing_data(map_p true_map, map_p modify_map)
{
    grid_cell_p cell;
    int num_changed_points;
    int index;
    double value;
    int list_index;

    current_data_set = (current_data_set+1) % LASER_HISTORY_LENGTH;

    if (laser_scan[current_data_set] == NULL) {
        laser_scan[current_data_set] =
                (grid_cell_p)calloc(200, sizeof(grid_cell_t));
        assert(laser_scan[current_data_set]);
        max_scan_size[current_data_set] = 200;
        scan_size[current_data_set] = 0;
    }

    /* The oldest scan is erased here */
    cell = laser_scan[current_data_set];

    num_changed_points = scan_size[current_data_set];
    cell = laser_scan[current_data_set];
    for (index = 0; index < num_changed_points; index++) {
        value = true_map->map[cell->x][cell->y];
        modify_map->map[cell->x][cell->y] = value;
        cell++;
    }
    scan_size[current_data_set] = 0;

    /* The other scans are recopied in here */

    for (list_index = 0; list_index < LASER_HISTORY_LENGTH; list_index++) {
        cell = laser_scan[list_index];
        if (cell == NULL)
            continue;
        num_changed_points = scan_size[list_index];
        for (index = 0; index < num_changed_points; index++) {
            modify_map->map[cell->x][cell->y] = cell->value;
            /*        warn("Filling point %d of %d in list %d: %d %d\n", index, */
            /*  	   num_changed_points, list_index, cell->x, cell->y); */
            cell++;
        }
    }
}

void 
trace_laser(int x_1, int y_1, int x_2, int y_2, map_p true_map,
            map_p modify_map)
{
    bresenham_param_t params;
    int X, Y;
    double true_map_value;
    double modified_map_value;

    get_bresenham_parameters(x_1, y_1, x_2, y_2, &params);

    do {
        get_current_point(&params, &X, &Y);
        if (!is_in_map(X, Y, modify_map))
            break;
        true_map_value = true_map->map[X][Y];
        modified_map_value = modify_map->map[X][Y];

        if (!is_empty(modified_map_value)  &&   is_empty(true_map_value))
            add_clear_point(X, Y, true_map, modify_map);
    } while (get_next_point(&params));
}

void 
map_modify_update(robot_laser_message *laser_msg,
                  navigator_config_t *config,
                  world_point_p world_point,
                  map_p true_map,
                  map_p modify_map)
{
    int index;
    double angle, separation;
    int laser_x, laser_y;
    double dist;
    int increment;
    map_point_t map_point;
    int count;

    int maxrange_beam;

    if (!config->map_update_freespace &&  !config->map_update_obstacles)
        return;

    if (true_map == NULL || modify_map == NULL)
    {
        //warn("%s called with NULL map argument.\n", __FUNCTION__);
        return;
    }

    if (laser_scan == NULL) {
        laser_scan = (grid_cell_p *)calloc(LASER_HISTORY_LENGTH, sizeof(grid_cell_p));
        assert(laser_scan);

        scan_size = (int *)calloc(LASER_HISTORY_LENGTH, sizeof(int));
        assert(scan_size);

        max_scan_size = (int *)calloc(LASER_HISTORY_LENGTH, sizeof(int));
        assert(max_scan_size);
    }

    update_existing_data(true_map, modify_map);

    if (laser_msg->num_readings < config->num_lasers_to_use)   {
        increment = 1;
        separation = laser_msg->config.angular_resolution;
    }
    else  {
        increment = laser_msg->num_readings / config->num_lasers_to_use;
        if (increment != 1)
            separation = normalize_theta(laser_msg->config.fov / (config->num_lasers_to_use+1));
        else
            separation = laser_msg->config.angular_resolution;
    }

    angle = normalize_theta(world_point->pose.theta +
                                   laser_msg->config.start_angle);
    world_to_map(world_point, &map_point);

    count = 0;
    for (index = 0; index < laser_msg->num_readings; index += increment) {

        if (laser_msg->range[index] < laser_msg->config.maximum_range  )
            maxrange_beam = 0;
        else
            maxrange_beam = 1;

        if (config->map_update_freespace) {

            dist = laser_msg->range[index] - modify_map->config.resolution;
            if (dist < 0)
                dist = 0;
            if (dist > config->map_update_radius)
                dist = config->map_update_radius;

            laser_x = round(map_point.x + (cos(angle)*dist)/
                                   modify_map->config.resolution);
            laser_y = round(map_point.y + (sin(angle)*dist)/
                                   modify_map->config.resolution);

            trace_laser(map_point.x, map_point.y, laser_x, laser_y, true_map, modify_map);
        }

        if (config->map_update_obstacles) {

            // add obstacle only if it is not a maxrange reading!
            if (!maxrange_beam) {

                dist = laser_msg->range[index];


                laser_x = round(map_point.x + (cos(angle)*dist)/
                                       modify_map->config.resolution);
                laser_y = round(map_point.y + (sin(angle)*dist)/
                                       modify_map->config.resolution);

                if (is_in_map(laser_x, laser_y, modify_map) &&
                        dist < config->map_update_radius &&
                        dist < (laser_msg->config.maximum_range - 2*modify_map->config.resolution)) {
                    count++;
                    add_filled_point(laser_x, laser_y, true_map, modify_map);
                }
            }
        }
        angle += separation;
    }
}

void 
map_modify_clear(map_p true_map, map_p modify_map)
{
    int index;

    if (scan_size == NULL)
        return;

    for (index = 0; index < LASER_HISTORY_LENGTH; index++)
        scan_size[index] = 0;

    memcpy(modify_map->complete_map, true_map->complete_map,
           true_map->config.x_size*true_map->config.y_size*sizeof(float));

	//crti:2016-12-20,navmap
	memcpy(modify_map->complete_navmap, true_map->complete_navmap,
           true_map->config.x_size*true_map->config.y_size*sizeof(float));
    
    current_data_set = 0;
}
