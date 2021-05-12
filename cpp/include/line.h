#ifndef LINE_H
#define LINE_H

#include "Eigen/Dense"
#include "helper.h"

class Line {
private:
    Eigen::Vector2d start_point;
    Eigen::Vector2d end_point;
    Eigen::Vector2d line_vec;
    BoundingBox bounding_box;

public:
    Line(const Eigen::Vector2d & start_point, const Eigen::Vector2d & end_point) :
    start_point(start_point), end_point(end_point), line_vec(end_point - start_point), bounding_box(start_point, end_point) {};
    
    inline Eigen::Vector2d get_start() const { return start_point; }
    inline Eigen::Vector2d get_end() const { return end_point; }
    inline Eigen::Vector2d get_line_vec() const { return line_vec; }

    inline BoundingBox get_bounding_box() const { return bounding_box; }
};

#endif