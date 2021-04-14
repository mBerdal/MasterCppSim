#ifndef ENV_H
#define ENV_H

#include "Eigen/Dense"
using namespace Eigen;

#include <list>
#include <vector>

class Wall {
private:
    Vector2d start_point;
    Vector2d end_point;

public:
    Wall(Vector2d start_point, Vector2d end_point) : start_point(start_point), end_point(end_point) {}
    inline Vector2d start() const { return start_point; }
    inline Vector2d end() const { return end_point; }
};

class Env {
private:
    std::vector<Wall> walls;

public:
    Env() : walls(std::vector<Wall>()) {}
    Env(std::list<Matrix<double, 2, Dynamic>> obstacles);
    std::vector<Wall> get_walls() const { return walls; }
};

#endif