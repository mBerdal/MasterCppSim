#ifndef ENV_H
#define ENV_H

#include <Eigen/Dense>
#include <list>
#include <vector>

class Wall {
private:
    Eigen::Vector2d start_point;
    Eigen::Vector2d end_point;

public:
    Wall(Eigen::Vector2d start_point, Eigen::Vector2d end_point) : start_point(start_point), end_point(end_point) {}
    inline Eigen::Vector2d get_start() const { return start_point; }
    inline Eigen::Vector2d get_end() const { return end_point; }
};

class Env {
private:
    std::vector<Wall> walls;
public:
    Env() : walls(std::vector<Wall>()) {}
    Env(std::vector<Wall> walls) : walls(walls) {}
    Env(std::list<Eigen::Matrix<double, 2, Eigen::Dynamic>> obstacles);
    inline std::vector<Wall> get_walls() const { return walls; }

    static Env ten_by_ten;
};

#endif