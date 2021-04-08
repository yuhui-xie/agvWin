
#include <assert.h>

//#include "navigator.h"
#include "conventional.h" 

#define NUM_ACTIONS 8

extern map_t * planner_map;
extern int planner_x_offset[NUM_ACTIONS] ;//= {0,   1, 1, 1, 0, -1, -1, -1,  0,  1, 1, 1, 0, -1, -1, -1};
extern int planner_y_offset[NUM_ACTIONS] ;//= {-1, -1, 0, 1, 1,  1,  0, -1, -1, -1, 0, 1, 1, 1, 0, -1};

#define MAX_UTILITY 1000.0
/* How much to reduce the cost per meter.
   Kind of arbitrary, but related to MAX_UTILITY */
#define MIN_COST 0.1

struct state_struct {
  int x, y;
  int entropy;
  double cost;
  int due_to_sensing;
};

typedef struct state_struct state;
typedef state *state_ptr;

typedef struct {
  state_ptr *data_array;
  int num_elements;
  int queue_size;
} queue_struct, *queue;

static int x_size, y_size;

static double *costs = NULL;
static double *utility = NULL;

inline double clamp(double X, double Y, double Z)
{
  if (Y < X)
    return X;
  else if (Y > Z)
    return Z;
  return Y;
}
inline static int
is_out_of_map(int x, int y)
{
  return (x >= x_size || x < 0 || y >= y_size || y < 0);
}

inline static
double *utility_value(int x, int y)
{
  if (is_out_of_map(x, y))
    return NULL;

  return (utility + x*y_size + y);
}

static inline double
get_parent_value(queue the_queue, int index)
{
  int parent_index;
  int x, y;

  parent_index = (int)(index/2)-1;
  x = the_queue->data_array[parent_index]->x;
  y = the_queue->data_array[parent_index]->y;

  return *utility_value(x, y);
}

static inline queue
make_queue(void) 
{
  queue new_queue;
  
  new_queue=(queue)calloc(1, sizeof(queue_struct));
  assert(new_queue);

  new_queue->data_array=(state_ptr *)calloc(1, sizeof(state_ptr));
  assert(new_queue->data_array);

  return new_queue;
}

static inline void
swap_entries(int x1, int x2, queue the_queue)
{
  state_ptr tmp;
  
  tmp = the_queue->data_array[x1-1];
  the_queue->data_array[x1-1] = the_queue->data_array[x2-1];
  the_queue->data_array[x2-1] = tmp;
}

static inline void
fix_priority_queue(queue the_queue)
{
  int left, right;
  int index;
  int largest;

  index = 1;

  while (index < the_queue->num_elements) 
    {      
      left = 2*index;
      right = 2*index+1;
      
      if (left <= the_queue->num_elements && 
	  the_queue->data_array[left-1]->cost > the_queue->data_array[index-1]->cost)
	largest = left;
      else
	largest = index;
      if (right <= the_queue->num_elements && 
	  the_queue->data_array[right-1]->cost > the_queue->data_array[largest - 1]->cost)
	largest = right;
      
      if (largest != index)
	{
	  swap_entries(largest, index, the_queue);
	  index = largest;
	}
      else
	break;
    }
}

static inline state_ptr
pop_queue(queue the_queue) 
{
  state_ptr return_state;
  
  if (the_queue->num_elements == 0)
    return NULL;
  
  return_state = the_queue->data_array[0];
  
  the_queue->data_array[0] = the_queue->data_array[the_queue->num_elements-1];
  the_queue->num_elements--;
  
  fix_priority_queue(the_queue);
  
  return(return_state);
}

static inline void
delete_queue(queue *queue_pointer) 
{
  queue the_queue;
  state_ptr cur_state;
  
  the_queue = *queue_pointer;

  if (the_queue == NULL)
    return;

  while (the_queue->num_elements > 0) 
    {
      cur_state = pop_queue(the_queue);
      free(cur_state);
    }
  
  if (the_queue->queue_size > 0)
    free(the_queue->data_array);
  
  free(the_queue);
  queue_pointer = NULL;
}

static void
resize_queue(queue the_queue)
{
  if (the_queue->queue_size == 0) 
    {
      the_queue->data_array=(state_ptr *)calloc(256, sizeof(state_ptr));
      assert(the_queue->data_array);
      the_queue->queue_size = 256;
      the_queue->num_elements = 0;
      return;
    }

  /* If the queue is full, we had better grow it some. */

  if (the_queue->queue_size < the_queue->num_elements) 
    return ;

  /* Realloc twice as much space */
      
  the_queue->data_array=(state_ptr *)
    realloc(the_queue->data_array, sizeof(state_ptr)*the_queue->queue_size*2);
  assert(the_queue->data_array);
  
  the_queue->queue_size *= 2;
  memset(the_queue->data_array+the_queue->num_elements, 0, 
	 (the_queue->queue_size - the_queue->num_elements)*sizeof(state_ptr));
}

static inline void
insert_into_queue(state_ptr new_state, queue the_queue) 
{
  int index;

  if (!the_queue->queue_size || 
      the_queue->queue_size == the_queue->num_elements) 
    resize_queue(the_queue);

  the_queue->data_array[the_queue->num_elements] = new_state;
  the_queue->num_elements++;

  /* Fix up priority queue */

  index = the_queue->num_elements;
  
  while (index > 1 && get_parent_value(the_queue, index) < new_state->cost) 
    {
      swap_entries((int)(index/2), index, the_queue);
      index = (int)(index/2);
    }
}

void conventional_build_costs(float robotWidth,
                     map_point_t *robot_posn,
                     navigator_config_t *navigator_conf)
{
  int x_index = 0, y_index = 0;
  double value;
  double *cost_ptr;
  float *map_ptr;
  double resolution;
  int index;
  int x, y;
  int x_start, y_start;
  int x_end, y_end;
  double robot_distance;


  if (x_size != planner_map->config.x_size ||
      y_size != planner_map->config.y_size) {
    free(costs);
    costs = NULL;
    free(utility);
    utility = NULL;
  }

  x_size = planner_map->config.x_size;
  y_size = planner_map->config.y_size;

  if (costs == NULL) {
    costs = (double *)calloc(x_size*y_size, sizeof(double));
    assert(costs);
    for (index = 0; index < x_size*y_size; index++)
      costs[index] = planner_map->complete_map[index];
  }

  resolution = planner_map->config.resolution;

  if (robot_posn && navigator_conf) {
    x_start = robot_posn->x - navigator_conf->map_update_radius/
      robot_posn->map->config.resolution;
    x_start = clamp(0, x_start, x_size);
    y_start = robot_posn->y - navigator_conf->map_update_radius/
      robot_posn->map->config.resolution;
    y_start = clamp(0, y_start, y_size);
    x_end = robot_posn->x + navigator_conf->map_update_radius/
      robot_posn->map->config.resolution;
    x_end = clamp(0, x_end, x_size);
    y_end = robot_posn->y + navigator_conf->map_update_radius/
      robot_posn->map->config.resolution;
    y_end = clamp(0, y_end, y_size);
    
    cost_ptr = costs+x_start*y_size;
    map_ptr = planner_map->complete_map+x_start*y_size;
    for (x_index = x_start; x_index < x_end; x_index++) {
      for (y_index = y_start; y_index < y_end; y_index++)
	cost_ptr[y_index] = map_ptr[y_index];
      cost_ptr += y_size;
      map_ptr += y_size;
    }      
  } else {
    x_start = 0;
    y_start = 0;
    x_end = x_size;
    y_end = y_size;
    
    for (index = 0; index < x_size*y_size; index++) 
      costs[index] = planner_map->complete_map[index];
  }
 //   //crti:2016-12-28
 //   //crti:2016-12-20,navmap
  //HXW.3.28
        for (index = 0; index < x_size*y_size; index++)
		{
		float navtmp = planner_map->complete_navmap[index];
		if(navtmp>0.5)
		{
			costs[index] = navtmp;
		}
	    }

  /* Initialize cost function to match map, where empty cells have
     MIN_COST cost and filled cells have MAX_UTILITY cost */

  for (x_index = x_start; x_index < x_end; x_index++) {
    cost_ptr = costs+x_index*y_size+y_start;
    for (y_index = y_start; y_index < y_end; y_index++) {
      value = *(cost_ptr);
      if (value >= 0 && value < MIN_COST)
	value = MIN_COST;
      else
	value = MAX_UTILITY;
      *(cost_ptr++) = value;
    }
  }
  
  /* Loop through cost map, starting at top left, updating cost of cell
     as max of itself and most expensive neighbour less the cost 
     downgrade. */

  for (x_index = x_start; x_index < x_end; x_index++) {
    cost_ptr = costs+x_index*y_size+y_start;
    for (y_index = y_start; y_index < y_end; y_index++, cost_ptr++) {
      if (x_index < 1 || x_index >= x_size-1 || y_index < 1 || 
	  y_index >= y_size-1) 
	continue;
      
      for (index = 0; index < NUM_ACTIONS; index++) {
    x = x_index + planner_x_offset[index];
    y = y_index + planner_y_offset[index];
	
	value = *(costs+x*y_size+y) - resolution*MAX_UTILITY;
	if (value > *cost_ptr) 
	  *cost_ptr = value; 
      }
    }
  }

  /* Loop through cost map again, starting at bottom right, updating cost of 
     cell as max of itself and most expensive neighbour less the cost 
     downgrade. */

  for (x_index = x_end-1; x_index >= x_start; x_index--) {
    cost_ptr = costs+x_index*y_size+y_end-1;
    for (y_index = y_end-1; y_index >= y_start; y_index--, cost_ptr--) {
      if (x_index < 1 || x_index >= x_size-1 || y_index < 1 || 
	  y_index >= y_size-1) 
	continue;
      
      for (index = 0; index < NUM_ACTIONS; index++) {
    x = x_index + planner_x_offset[index];
    y = y_index + planner_y_offset[index];
	
	value = *(costs+x*y_size+y) - resolution*MAX_UTILITY;
	if (value > *cost_ptr) 
	  *cost_ptr = value; 
      }
    }
  }
  //robot_distance = robotWidth * MAX_UTILITY;
  robot_distance = robotWidth * MAX_UTILITY;
  //robot_distance = 0.05*MAX_UTILITY;
  /* Clamp any cell that's closer than the robot width to be 
     impassable. Also, rescale cost function so that the max cost
     is 0.5 */
  //HXW.4.17 
  for (x_index = x_start; x_index < x_end; x_index++) {
    cost_ptr = costs+x_index*y_size+y_start;
    for (y_index = y_start; y_index < y_end; y_index++) {
      value = *(cost_ptr);
      if (value < MAX_UTILITY - robot_distance) {
		  value = value / (4*MAX_UTILITY);//crti:2016-05-26,ԭ/ (2*MAX_UTILITY);
		    //value = value / (1.8* MAX_UTILITY);
		  if (value < MIN_COST) value = MIN_COST;
	  }
	  //else if (value <MAX_UTILITY - robot_distance*0.5 && value >MAX_UTILITY - robot_distance)
		  //value = value / (2 * MAX_UTILITY); 
	  else
		  value = 1.0;
	  *(cost_ptr++) = value;
    }
  }
  
  if (robot_posn != NULL) {
    x_index = robot_posn->x / planner_map->config.resolution;
    y_index = robot_posn->y / planner_map->config.resolution;
    
    if (x_index >= 0 && x_index < x_size && y_index >= 0 && y_index < y_size)
      *(costs+x_index*y_size+y_index) = MIN_COST;
  }

  //crti:2016-12-20,navmap
  //HXW.3.28进行注释
    //for (index = 0; index < x_size*y_size; index++){
		//float navtmp = planner_map->complete_navmap[index];
		//if(navtmp>0.5){
			//costs[index] = navtmp;
		//}
	//}

}

double 
conventional_get_cost(int x, int y)
{
  if (is_out_of_map(x, y))
    return -1;
  return *(costs+x*y_size + y);
}

double 
conventional_get_utility(int x, int y)
{
  if (!utility)
    return -1;
  if (is_out_of_map(x, y))
    return -1;
  return *(utility+x*y_size + y);
}

double *
conventional_get_utility_ptr(void)
{
  return utility;
}

double *
conventional_get_costs_ptr(void)
{
  return costs;
}

static inline state_ptr
conventional_create_state(int x, int y, double util)
{
  state_ptr new_ptr = NULL;

  if (is_out_of_map(x, y))
      exit(0);

  new_ptr = (state_ptr)calloc(1, sizeof(state));
  assert(new_ptr);

  new_ptr->x = x;
  new_ptr->y = y;
  new_ptr->cost = util;

  return new_ptr;
}

static inline void
push_state(int x, int y, double new_utility, queue state_queue)
{
  state_ptr new_state = conventional_create_state(x, y, new_utility);
  insert_into_queue(new_state, state_queue);
  *(utility_value(x, y)) = new_utility;
} 

static inline void
add_neighbours_to_queue(int x, int y, queue state_queue)
{
  int index;
  double cur_util, new_util, multiplier;
  int cur_x, cur_y;
  double parent_utility;
  
  parent_utility = *utility_value(x, y);

  for (index = 0; index < NUM_ACTIONS; index+=2) 
    {      
      if (index % 2 == 1)
	multiplier = M_SQRT2;
      else
	multiplier = 1;
      
      cur_x = x + planner_x_offset[index];
      cur_y = y + planner_y_offset[index];

      if (is_out_of_map(cur_x, cur_y) || *(costs+cur_x*y_size + cur_y) > 0.5)
	continue;
      
      cur_util = *(utility_value(cur_x, cur_y));
      new_util = parent_utility - *(costs+cur_x*y_size + cur_y)*multiplier;
      assert(new_util > 0);
      if (cur_util < 0)
	push_state(cur_x, cur_y, new_util, state_queue);
    } /* End of for (Index = 0...) */
  
}

void 
conventional_dynamic_program(int goal_x, int goal_y)
{
  double *utility_ptr;
  int index;
  double max_val, min_val;
  int num_expanded;
  int done;

  //struct timeval start_time, end_time;
  int delta_sec, delta_usec;
  
  queue state_queue;
  state_ptr current_state;

  if (costs == NULL)
    return;

  //gettimeofday(&start_time, NULL);


  if (utility == NULL) {
    utility = (double *)calloc(x_size*y_size, sizeof(double));
    assert(utility);
  }
  
  utility_ptr = utility;
  for (index = 0; index < x_size * y_size; index++) 
    *(utility_ptr++) = -1;

  if (is_out_of_map(goal_x, goal_y))
    return;

  max_val = -MAXDOUBLE;
  min_val = MAXDOUBLE; 
  done = 0;

  state_queue = make_queue();

  current_state = conventional_create_state(goal_x, goal_y, 0);
  max_val = MAX_UTILITY;
  *(utility_value(goal_x, goal_y)) = max_val;
  add_neighbours_to_queue(goal_x, goal_y, state_queue);    
  num_expanded = 1;
  
  while ((current_state = pop_queue(state_queue)) != NULL) {
    num_expanded++;
    if (current_state->cost <= *(utility_value(current_state->x, current_state->y)))
      add_neighbours_to_queue(current_state->x, current_state->y, state_queue);
    if (current_state->cost < min_val)
      min_val = current_state->cost;
    free(current_state);
  }

  delete_queue(&state_queue);

  //gettimeofday(&end_time, NULL);



  //  warn("Elasped time for dp: %d secs, %d usecs\n", delta_sec, delta_usec);
}

void 
conventional_find_best_action(map_point_p curpoint)
{
  double best_utility;
  int best_x = curpoint->x;
  int best_y = curpoint->y;
  int new_x, new_y;
  int index;
  double util = 0.0;

  if (utility == NULL)
    return;

  best_utility = *(utility_value(best_x, best_y));
  for (index = 0; index < NUM_ACTIONS; index++) 
    {
      new_x = curpoint->x + planner_x_offset[index];
      new_y = curpoint->y + planner_y_offset[index];
      
      if (new_x < 0 || new_x >= planner_map->config.x_size ||
      new_y < 0 || new_y >= planner_map->config.y_size)
	continue;
      util = *(utility_value(new_x, new_y));
      if (util >= 0.0 && util > best_utility) 
	{
	  best_x = new_x;
	  best_y = new_y;
	  best_utility = util;
	}
    }

  curpoint->x = best_x;
  curpoint->y = best_y;

  return;
}

void 
conventional_end_planner(void)
{
  if (costs != NULL)
    free(costs);
  if (utility != NULL)
    free(utility);
  
}

