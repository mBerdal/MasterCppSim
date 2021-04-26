#include "helper.h"
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

double cross(eig::Matrix<double, 2, 1> a, eig::Matrix<double, 2, 1> b) {
    return a(0) * b(1) - a(1) * b(0);
}

std::vector<double> eig_vec2std_vec(eig::VectorXd vec) {
    return std::vector<double>(vec.data(), vec.data() + vec.size());
}

eig::VectorXd clamp_vec (eig::VectorXd vec, double threshold) {
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
