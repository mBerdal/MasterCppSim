#include "helper.h"

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

double cross(Matrix<double, 2, 1> a, Matrix<double, 2, 1> b) {
    return a(0) * b(1) - a(1) * b(0);
}

std::vector<double> eig_vec2std_vec(Vector<double, Dynamic> vec) {
    return std::vector<double>(vec.data(), vec.data() + vec.size());
}

Vector<double, Dynamic> clamp_vec (Vector<double, Dynamic> vec, double threshold) {
    double norm = vec.norm();
    if (norm < threshold) {
        return vec;
    }
    return (threshold / norm) * vec;
}
