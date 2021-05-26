#ifndef ENV_H
#define ENV_H

#include <Eigen/Dense>
#include <list>
#include <vector>
#include <nlohmann/json.hpp>

#include "line.h"
#include "helper.h"

class Env {
private:
    std::vector<Line> walls;
    std::string name;
    BoundingBox bounding_box;

public:
    Env(const std::string & name="noname");
    Env(const std::vector<Line> & walls, const std::string & name="noname");
    Env(const std::list<Eigen::Matrix<double, 2, Eigen::Dynamic>> & obstacles, const std::string & name="noname");
    
    inline std::vector<Line> get_walls() const { return walls; }
    inline std::string get_name() const { return name; }
    inline BoundingBox get_bounding_box() const { return bounding_box; }
    inline bool get_is_open () const { return walls.size() == 0; }

    nlohmann::json add_to_json(nlohmann::json & j) const;
    
    static Env ten_by_ten;
    static Env fifty_by_fifty;
    static Env stripa_short;
    static Env stripa;
};

#endif