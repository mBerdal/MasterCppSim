#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>   /* cos */
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */

#include "Eigen/Dense"
using namespace Eigen;

template <class T>
class RandomNumberGenerator {
public:
    inline static void seed() { srand(time(0)); }
    inline static T get_between(T low, T high) {
        T random = ((T)rand()) / (T)RAND_MAX;
        T range = high - low;
        return (random * range) + low;
    }
};

template <class T>
T get_Xi_from_model(T d, T d_min, T d_max, T xi_bar) {
    if (d < d_min){
        return xi_bar;
    }
    if (d > d_max) {
        return 0;
    }
    return (T) (xi_bar / (T) 2)*(1 + cos((M_PI / (d_max - d_min)) * (d - d_max)));
}

template <class T>
void euler_int(Matrix<T, Dynamic, 1>* x, Matrix<T, Dynamic, 1> x_dot, T dt) {
    *x += dt*x_dot;
}

template<class T>
T cross(Matrix<T, 2, 1> a, Matrix<T, 2, 1> b) {
    return a(0) * b(1) - a(1) * b(0);
}

template<class T>
std::vector<T> eig_mat2std_vec(Matrix<T, Dynamic, Dynamic> mat) {
    return std::vector<T>(mat.data(), mat.data() + mat.size());
}

#endif