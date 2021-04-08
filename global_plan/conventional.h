


#ifndef CONVENTIONAL_H
#define CONVENTIONAL_H

#include "data.h"

#ifdef __cplusplus
extern "C" {
#endif

  /** Computes the utility function using dynamic programming. 
      conventional_build_costs must have been
      called first. **/ 
  void conventional_dynamic_program(int goal_x, int goal_y);
  /** Takes in the current position (as a map grid cell) and replaces
      the argument with the best neighbour grid cell to visit
      next. conventional_dynamic_program must have been
      called first. **/ 
  void conventional_find_best_action(map_point_p curpoint);
  /** Frees memory structures. **/ 
  void conventional_end_planner(void);
  /** Returns the utility function as an array of doubles with the
   same dimensions as the map in row-major order. **/ 
  double *conventional_get_utility_ptr(void);
  /** Returns the cost map as an array of doubles with the
   same dimensions as the map in row-major order. **/ 
  double *conventional_get_costs_ptr(void);
  /** Returns the value of the utility function at a specific point,
   that is, the cost-to-goal. **/ 
  double conventional_get_utility(int x, int y);
  /** Returns the value of the cost map at a specific point. **/ 
  double conventional_get_cost(int x, int y);
  /** Converts the occupancy grid map into a cost map by incorporating
      the width of the robot, and adding additional penalties for getting
      close to obstacles. **/ 
  void conventional_build_costs(float robotWidht,
                       map_point_t *robot_posn,
                       navigator_config_t *navigator_conf);

#ifdef __cplusplus
}
#endif

#endif
// @}
