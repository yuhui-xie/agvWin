
#ifndef MAP_MODIFY_H
#define MAP_MODIFY_H
#include "data.h"

#ifdef __cplusplus
extern "C" {
#endif

  void map_modify_update(robot_laser_message *laser_msg,
             navigator_config_t *navigator_config,
             world_point_p world_point,
             map_p true_map, map_p modify_map);
  void map_modify_clear(map_p true_map, map_p modify_map);

#ifdef __cplusplus
}
#endif

#endif
