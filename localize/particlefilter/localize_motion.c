
#include "localize_motion.h"


localize_motion_model_t *localize_motion_initialize(int argc, char *argv[])
{
    localize_motion_model_t *model;

    model = (localize_motion_model_t *)
            calloc(1, sizeof(localize_motion_model_t));
    //test_alloc(model);

    //install_params(model, argc, argv);

    return model;
}

double localize_sample_noisy_downrange(double delta_t,
                                       double delta_theta,
                                       localize_motion_model_t
                                       *model)
{
    double downrange_mean, downrange_std_dev;
    double sample;

    downrange_mean = delta_t*model->mean_d_d+delta_theta*model->mean_d_t;
    downrange_std_dev =fabs(delta_t)*model->std_dev_d_d+fabs(delta_theta)*model->std_dev_d_t;

    if (downrange_std_dev < 1e-6)
        return downrange_mean;

    do {
        sample = gaussian_random(downrange_mean, downrange_std_dev);
    } while (fabs(sample - downrange_mean) > 2*downrange_std_dev);

    return sample;
}

double localize_sample_noisy_crossrange(double delta_t,
                                        double delta_theta,
                                        localize_motion_model_t
                                        *model)
{
    double crossrange_mean, crossrange_std_dev;
    double sample;

    crossrange_mean = delta_t*model->mean_c_d+delta_theta*model->mean_c_t;
    crossrange_std_dev = fabs(delta_t)*model->std_dev_c_d+
            fabs(delta_theta)*model->std_dev_c_t;

    if (crossrange_std_dev < 1e-6)
        return crossrange_mean;

    do {
        sample = gaussian_random(crossrange_mean, crossrange_std_dev);
    } while (fabs(sample - crossrange_mean) > 2*crossrange_std_dev);

    return sample;
}

double localize_sample_noisy_turn(double delta_t, double delta_theta,
                                  localize_motion_model_t *model)
{
    double turn_mean, turn_std_dev;
    double sample;

    turn_mean = delta_t*model->mean_t_d+delta_theta*model->mean_t_t;
    turn_std_dev = fabs(delta_t)*model->std_dev_t_d+fabs(delta_theta)*model->std_dev_t_t;

    if (turn_std_dev < 1e-6)
        return turn_mean;

    do {
        sample = gaussian_random(turn_mean, turn_std_dev);
    } while (fabs(sample - turn_mean) > 2*turn_std_dev);

    return sample;
}
