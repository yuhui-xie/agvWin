
#include "trajectory.h"
inline double radians_to_degrees(double theta)
{
  return (theta * 180.0 / M_PI);
}
static void 
check_path_capacity(planner_path_p path)
{	
    traj_point_p new_points;

    if (path->capacity == 0) {
        path->capacity = 20;
        path->points = (traj_point_p)
                calloc(path->capacity, sizeof(traj_point_t));
        assert(path->points);

    } else if (path->length == path->capacity) {
        path->capacity *= 2;

        new_points = (traj_point_p)realloc
                (path->points, sizeof(traj_point_t)*path->capacity);
        assert(new_points);
        path->points = new_points;
    }
}

static void 
update_orientation(int index, planner_path_p path)
{
    traj_point_p point, next_point;

    if (index < 0 || index > path->length-2)
        return;

    point = path->points+index;
    next_point = path->points+index+1;

    point->theta = atan2(next_point->y - point->y, next_point->x - point->x);
}

int 
planner_util_add_path_point(traj_point_t point,
                                   planner_path_p path)
{
    check_path_capacity(path);

    path->points[path->length] = point;

    path->length++;

    update_orientation(path->length-2, path);

    return path->length-1;
}

traj_point_p
planner_util_get_path_point(int index, planner_path_p path)
{
    if (index >= path->length || index < 0)
    {
        return NULL;
    }

    return path->points+index;
}

void 
planner_util_set_path_point(int index, traj_point_p path_point,
                                   planner_path_p path)
{  
    if (index >= path->length || index < 0)
    {
        return;
    }
    path->points[index] = *path_point;

    update_orientation(index-1, path);

    return;
}

void 
planner_util_insert_blank(int index, planner_path_p path)
{
    int num_to_move;

    check_path_capacity(path);

    num_to_move = path->length - index;
    memmove(path->points+index+1, path->points+index,
            num_to_move*sizeof(traj_point_t));

    path->length++;
}

void 
planner_util_insert_path_point(int index,
                                      traj_point_t *current_point,
                                      planner_path_p path)
{
    planner_util_insert_blank(index, path);
    planner_util_set_path_point(index, current_point, path);
}

void 
planner_util_set_path_velocities(int index, double t_vel,
                                        double r_vel, planner_path_p path)
{
    if (index >= path->length || index < 0)
    {
        return;
    }
    path->points[index].t_vel = t_vel;
    path->points[index].r_vel = r_vel;

    return;
}

void 
planner_util_clear_path(planner_path_p path)
{
    path->length = 0;
}

void 
planner_util_clip_path(int length, planner_path_p path)
{
    path->length = length;
}

void 
planner_util_delete_path_point(int index, planner_path_p path)
{
    int num_to_move;

    if (index >= path->length || index < 0)
    {
        return;
    }

    num_to_move = path->length - index - 1;
    memmove(path->points+index,
            path->points+index+1, num_to_move*sizeof(traj_point_t));

    path->length--;
}

void
planner_util_test_trajectory(planner_path_p path)
{
    int index;
    traj_point_t point = {0,0,0,0,0};
    traj_point_p point_p;

    planner_util_clear_path(path);
    if (path->length != 0)
    {
        printf("Test failed: clear should set length to 0, but length is %d\n",
               path->length);
        exit(0);
    }

    memset(&point, 0, sizeof(traj_point_t));
    for (index = 0; index < 100; index++)
    {
        point.x = index;
        point.y = index;
        planner_util_add_path_point(point, path);
    }

    for (index = 0; index < 100; index++)
    {
        point_p = planner_util_get_path_point(index, path);
        if (point_p->x != index || point_p->y != index)
        {
            printf("Test failed: After 100 set points, get point on %d did not "
                       "match: was %.0f %.0f\n", index, point_p->x, point_p->y);
            exit(0);
        }
    }

    point.x = 50.5;
    point.y = 50.5;

    planner_util_insert_blank(50, path);
    planner_util_set_path_point(50, &point, path);
    point_p = planner_util_get_path_point(50, path);
    if (fabs(point_p->x - 50.5) > 0.05 || fabs(point_p->y - 50.5) > 0.05)
    {
        printf("Blank then set failed.\n");
        exit(0);
    }

    if (path->length != 101)
    {
        printf("Length (%d) not 101 after insert_blank then set.\n",
                   path->length);
        exit(0);
    }

    point.x = 60.5;
    point.y = 60.5;
    planner_util_insert_path_point(60, &point, path);

    if (fabs(point_p->x - 50.5) > 0.05 || fabs(point_p->y - 50.5) > 0.05)
    {
        printf("Blank then set failed.\n");
        exit(0);
    }

    if (path->length != 102)
    {
        printf("Length (%d) not 102 after insert_blank then set.\n",
                   path->length);
        exit(0);
    }

    planner_util_delete_path_point(50, path);
    planner_util_delete_path_point(60, path);

    if (path->length != 100)
    {
        printf("Length (%d) not 100 after insert_blank then set.\n",
                   path->length);
        exit(0);
    }

    planner_util_clip_path(50, path);

    for (index = 0; index < path->length; index++)
        planner_util_delete_path_point(index, path);

    for (index = 0; index < path->length; index++)
    {
        point_p = planner_util_get_path_point(index, path);
        if (point_p->x != 2*index+1 || point_p->y != 2*index+1)
        {
            printf("Test failed: After deleting even points, get point on %d "
                       "(%d) did not match: was %.0f %.0f %.0f\n", 2*index+1, index,
                       point_p->x, point_p->y,
                       radians_to_degrees(point_p->theta));
            exit(0);
        }
    }
}

#if MAKING_TEST
int main(int argc __attribute__ ((unused)), 
         char *argv[] __attribute__ ((unused)))
{
    planner_path_t path = {NULL, 0, 0};

    planner_util_test_trajectory(&path);

    return 0;
}
#endif
