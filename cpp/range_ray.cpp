#include "include/range_ray.h"
#include "include/helper.h"
using namespace Eigen;

bool is_between_zero_and_one(double val) {
    return val >= 0 && val <= 1;
}
bool is_between_zero_and_max_bound(double val, double max_bound){
    return val >= 0 && val <= max_bound;
}

Env RangeRay::env = Env();

double RangeRay::sense(Vector2d ray_origin, double ray_orientation, double max_range) {
    const Vector2d b = Rotation2D<double>(ray_orientation).toRotationMatrix() * Vector2d::UnitX();

    /*
    * TODO: Use bounding box to rule out walls too far away
    
        double c = cos(ray_orientation);
        double s = sin(ray_orientation);
        bounding_box_t ray_box = {
            .min_x = c <= 0 ? ray_origin(0) + max_range*c : ray_origin(0),
            .max_x = c <= 0 ? ray_origin(0) : ray_origin(0) + max_range*c,
            .min_y = s <= 0 ? ray_origin(0) + max_range*s : ray_origin(0),
            .max_y = s <= 0 ? ray_origin(0) : ray_origin(0) + max_range*s
        };
    */


    double min_sensed_range = max_range;

    for (Wall const &w : env.get_walls()) {
        const Vector2d a = w.get_end() - w.get_start();
        const double crs = cross(a, b);
        if (crs != 0) {
            const Vector2d v = (ray_origin - w.get_start()) / crs;
            double t = cross(v, b);
            double sensed_range = cross(v, a);
            if (is_between_zero_and_one(t) && is_between_zero_and_max_bound(sensed_range, max_range)) {
                min_sensed_range = std::min(min_sensed_range, sensed_range);
            }
        }
    }
    return min_sensed_range;
}