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

#endif
