
#ifndef PLANNER_H
#define PLANNER_H


#ifdef __cplusplus
extern "C" {
#endif
#include "data.h"
#include <assert.h>
#include "conventional.h"
#include "trajectory.h"
#include "map_modify.h"


int planner_update_robot(traj_point_p new_position,
                            navigator_config_t *nav_conf);

int planner_update_goal(point_p new_goal, int any_orientation,
                           navigator_config_t *nav_conf );

void planner_set_map(map_p new_map, float robotWidth );


void planner_reset_map(float robotWidth);

void planner_update_map(robot_laser_message *laser_msg,
                          navigator_config_t *nav_conf);

void planner_get_status(planner_status_p status) ;


#ifdef __cplusplus
}
#endif

#endif
