#ifndef HELPERS_H
#define HELPERS_H

#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include "Eigen/Dense"
using namespace Eigen;

#include <math.h>   /* cos */
#include <vector>

class RandomNumberGenerator {
public:
    inline static void seed() { srand(time(0)); }
    static double get_between(double low, double high);
};

double get_Xi_from_model(double d, double d_perf, double d_none, double xi_bar);

double cross(Matrix<double, 2, 1> a, Matrix<double, 2, 1> b);

std::vector<double> eig_vec2std_vec(Vector<double, Dynamic> vec);

Vector<double, Dynamic> clamp_vec (Vector<double, Dynamic> vec, double threshold);

template<typename T>
bool vectors_equal(std::vector<T> v1, std::vector<T> v2) {
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
double clamp_zero_pi(double ang);

class CircleSector {
private:
    double start_ang;
    double end_ang;
public:
    CircleSector(double start_ang, double end_ang) : start_ang(start_ang), end_ang(end_ang) {}
    inline double get_central_angle() const { return (end_ang > start_ang) ? end_ang - start_ang : end_ang - start_ang + 2 * M_PI; }
    inline double start() const {return start_ang; }
    inline double end() const {return end_ang; }
    inline double get_angle_bisector() const { return start_ang == end_ang ? start_ang : start_ang + 0.5 * get_central_angle(); }

    static bool cmp(const CircleSector & lhs, const CircleSector & rhs) { return lhs.get_central_angle() < rhs.get_central_angle(); }
};

#endif
