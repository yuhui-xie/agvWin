
#ifndef CTRAJECTORY_H
#define CTRAJECTORY_H

#include "data.h"
#ifdef __cplusplus
extern "C" {
#endif
  
  int planner_util_add_path_point(traj_point_t point,
                     planner_path_p path);
  
  traj_point_p planner_util_get_path_point(int index,
                             planner_path_p path);
  void planner_util_set_path_point(int index,
                      traj_point_p path_point,
                      planner_path_p path);
  void planner_util_insert_blank(int index, planner_path_p path);
  void planner_util_insert_path_point(int index,
                         traj_point_t *current_point,
                         planner_path_p path);
  void planner_util_set_path_velocities(int index, double t_vel,
					       double r_vel, 
                           planner_path_p path);
  void planner_util_clear_path(planner_path_p path);
  void planner_util_clip_path(int length, planner_path_p path);
  void planner_util_delete_path_point(int index, planner_path_p path);
  void planner_util_test_trajectory(planner_path_p path);
  
#ifdef __cplusplus
}
#endif

#endif
