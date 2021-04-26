#ifndef RANGE_RAY_H
#define RANGE_RAY_H

#include "Eigen/Dense"
#include "env.h"

class RangeRay {
public:
    static double sense(Eigen::Vector2d ray_origin, double ray_orientation, double max_range);
    static Env env;
};

#endif