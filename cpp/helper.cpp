#include "include/helper.h"
#include <iostream>
namespace eig = Eigen;

double RandomNumberGenerator::get_between(double low, double high) {
        double random = rand() / (double) RAND_MAX;
        double range = high - low;
        return (random * range) + low;
}

double get_Xi_from_model(double d, double d_perf, double d_none, double xi_bar) {
    if (d < d_perf){
        return xi_bar;
    }
    if (d > d_none) {
        return 0;
    }
    double omega = M_PI / (d_none - d_perf);
    double phi = - (M_PI * d_perf) / (d_none - d_perf);
    return (xi_bar / 2.0)*(1 + cos(omega * d + phi));
}

double cross(const eig::Matrix<double, 2, 1> & a, const eig::Matrix<double, 2, 1> & b) {
    return a(0) * b(1) - a(1) * b(0);
}

std::vector<double> eig_vec2std_vec(const eig::VectorXd & vec) {
    return std::vector<double>(vec.data(), vec.data() + vec.size());
}

eig::VectorXd clamp_vec (const eig::VectorXd & vec, double threshold) {
    double norm = vec.norm();
    if (norm < threshold) {
        return vec;
    }
    return (threshold / norm) * vec;
}

double wrap_max(double x, double max){
    return fmod(max + fmod(x, max), max);
}
double wrap_min_max(double x, double min, double max) {
    return min + wrap_max(x - min, max - min);
}

double clamp_pm_pi(double ang) {
    return wrap_min_max(ang, -M_PI, M_PI);
}

double clamp_zero_two_pi(double ang) {
    return wrap_min_max(ang, 0, 2 * M_PI);
}

bool BoundingBox::intersects (const BoundingBox & other) const {
    return !(
        min_x > other.max_x || max_x < other.min_x ||
        max_y < other.min_y || min_y > other.max_y
    );
}

void BoundingBox::print () const {
    std::cout << "Bounding box:\nmin_x: " << min_x << ", max_x: " << max_x << ", min_y: " << min_y << ", max_y: " << max_y;
}