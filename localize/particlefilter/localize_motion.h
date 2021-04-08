
/** @addtogroup localize liblocalize_motion **/
// @{

/** 
 * \file localize_motion.h
 * \brief Library for the new CARMEN motion_model.
 *
 * ...
 **/


#ifndef CARMEN_LOCALIZE_MOTION_H
#define CARMEN_LOCALIZE_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif
#include "data.h"

typedef struct {
    double mean_c_d;
    double mean_c_t;
    double std_dev_c_d;
    double std_dev_c_t;

    double mean_d_d;
    double mean_d_t;
    double std_dev_d_d;
    double std_dev_d_t;

    double mean_t_d;
    double mean_t_t;
    double std_dev_t_d;
    double std_dev_t_t;

	double alpha1;
	double alpha2;
	double alpha3;
	double alpha4;
	double alpha5;
} localize_motion_model_t;

localize_motion_model_t *localize_motion_initialize(int argc, char *argv[]);

double localize_sample_noisy_downrange(double delta_t,
                                       double delta_theta,
                                       localize_motion_model_t *model);

double localize_sample_noisy_crossrange(double delta_t,
                                        double delta_theta,
                                        localize_motion_model_t *model);

double localize_sample_noisy_turn(double delta_t,
                                  double delta_theta,
                                  localize_motion_model_t *model);

#ifdef __cplusplus
}
#endif

#endif
// @}
