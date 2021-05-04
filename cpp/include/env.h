#ifndef ENV_H
#define ENV_H

#include <Eigen/Dense>
#include <list>
#include <vector>

#include "helper.h"

class Wall {
private:
    Eigen::Vector2d start_point;
    Eigen::Vector2d end_point;
    bounding_box_t bounding_box;

public:
    Wall(const Eigen::Vector2d & start_point, const Eigen::Vector2d & end_point);
    inline Eigen::Vector2d get_start() const { return start_point; }
    inline Eigen::Vector2d get_end() const { return end_point; }

    inline bounding_box_t get_bounding_box() const { return bounding_box; }
};


class Env {
private:
    std::vector<Wall> walls;
    std::string name;
    bounding_box_t bounding_box;

public:
    Env(const std::string & name="noname");
    Env(const std::vector<Wall> & walls, const std::string & name="noname");
    Env(const std::list<Eigen::Matrix<double, 2, Eigen::Dynamic>> & obstacles, const std::string & name="noname");
    
    inline std::vector<Wall> get_walls() const { return walls; }
    inline std::string get_name() const { return name; }
    inline bounding_box_t get_bounding_box() const { return bounding_box; }
    
    static Env ten_by_ten;
    static Env stripa_short;
    static Env stripa;
};

#endif