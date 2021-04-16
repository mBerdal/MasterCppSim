#ifndef RANGE_RAY_H
#define RANGE_RAY_H

#include "Eigen/Dense"
#include "env.h"
using namespace Eigen;

class RangeRay {
public:
    static double sense(Vector2d ray_origin, double ray_orientation, double max_range);
    static Env env;
};

#endif