#include "include/range_ray.h"
#include "include/helper.h"
#include "include/line.h"

using namespace Eigen;

bool is_between_zero_and_one(double val) {
    return val >= 0 && val <= 1;
}
bool is_between_zero_and_max_bound(double val, double max_bound){
    return val >= 0 && val <= max_bound;
}

Env RangeRay::env = Env();
double RangeRay::max_range = 0;

double RangeRay::sense(const Vector2d & ray_origin, double ray_orientation) {
    const Vector2d b = Rotation2D<double>(ray_orientation).toRotationMatrix() * Vector2d::UnitX();

    double min_sensed_range = max_range;

    for (const Line & w : env.get_walls()) {
        const Vector2d a = w.get_line_vec();
        const double crs = cross(a, b);
        if (crs != 0) {
            const Vector2d v = (ray_origin - w.get_start()) / crs;
            double t = cross(v, b);
            double sensed_range = cross(v, a);
            if (is_between_zero_and_one(t) && is_between_zero_and_max_bound(sensed_range, max_range)) {
                min_sensed_range = min_sensed_range < sensed_range ? min_sensed_range : sensed_range;
            }
        }
    }
    return min_sensed_range;
}