#ifndef HELPERS_H
#define HELPERS_H

#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include <math.h>   /* cos */
#include <vector>

#include <Eigen/Dense>

class RandomNumberGenerator {
public:
    inline static void seed() { srand(time(0)); }
    static double get_between(double low, double high);
};

double get_Xi_from_model(double d, double d_perf, double d_none, double xi_bar);

double cross(const Eigen::Matrix<double, 2, 1> & a, const Eigen::Matrix<double, 2, 1> & b);

std::vector<double> eig_vec2std_vec(const Eigen::VectorXd & vec);

Eigen::VectorXd clamp_vec (const Eigen::VectorXd & vec, double threshold);

template<typename T>
bool vectors_equal(const std::vector<T> & v1, const std::vector<T> & v2) {
    int size = v1.size();
    if (size != v2.size()) {
        return false;
    }
    for (int i = 0; i < size; i++) {
        if (v1[i] != v2[i]) {
            return false;
        }
    }
    return true;
}

double clamp_pm_pi(double ang);
double clamp_zero_two_pi(double ang);

class CircleSector {
private:
    double start_ang, end_ang;
public:
    CircleSector(double start_ang, double end_ang) : start_ang(start_ang), end_ang(end_ang) {}
    inline double get_central_angle() const { return end_ang > start_ang ? end_ang - start_ang : end_ang - start_ang + 2 * M_PI; }
    inline double start() const {return start_ang; }
    inline double end() const {return end_ang; }
    inline double get_angle_bisector() const { return clamp_zero_two_pi(start_ang == end_ang ? start_ang : start_ang + 0.5 * get_central_angle()); }

    static bool cmp(const CircleSector & lhs, const CircleSector & rhs) { return lhs.get_central_angle() < rhs.get_central_angle(); }
};


class BoundingBox {
    private:
        double min_x, min_y, max_x, max_y;
    public:
        BoundingBox(Eigen::Vector2d line_start, Eigen::Vector2d line_end) :
        min_x(line_start(0) < line_end(0) ? line_start(0) : line_end(0)),
        min_y(line_start(1) < line_end(1) ? line_start(1) : line_end(1)),
        max_x(line_start(0) >= line_end(0) ? line_start(0) : line_end(0)),
        max_y(line_start(1) >= line_end(1) ? line_start(1) : line_end(1)) {};

        BoundingBox(double min_x = 0, double min_y = 0, double max_x = 0, double max_y = 0) :
        min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y) {};

        bool intersects (const BoundingBox & other) const;
        void print () const;

        inline double get_min_x() const { return min_x; }
        inline double get_min_y() const { return min_y; }
        inline double get_max_x() const { return max_x; }
        inline double get_max_y() const { return max_y; }
};

#endif
