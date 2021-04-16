#ifndef ENV_H
#define ENV_H

#include "Eigen/Dense"
using namespace Eigen;

#include <list>
#include <vector>
using namespace std;

class Wall {
private:
    Vector2d start_point;
    Vector2d end_point;

public:
    Wall(Vector2d start_point, Vector2d end_point) : start_point(start_point), end_point(end_point) {}
    inline Vector2d get_start() const { return start_point; }
    inline Vector2d get_end() const { return end_point; }
};

class Env {
private:
    vector<Wall> walls;
public:
    Env() : walls(vector<Wall>()) {}
    Env(list<Matrix<double, 2, Eigen::Dynamic>> obstacles);
    inline vector<Wall> get_walls() const { return walls; }
};  

#endif