#include "include/env.h"
namespace eig = Eigen;

Wall::Wall(eig::Vector2d start_point, eig::Vector2d end_point) : start_point(start_point), end_point(end_point) {
    bounding_box.min_x = start_point.x() < end_point.x() ? start_point.x() : end_point.x();
    bounding_box.max_x = start_point.x() > end_point.x() ? start_point.x() : end_point.x();
    bounding_box.min_y = start_point.y() < end_point.y() ? start_point.y() : end_point.y();
    bounding_box.max_y = start_point.y() > end_point.y() ? start_point.y() : end_point.y();
}

Env::Env(std::string name) : walls(std::vector<Wall>()), name(name), bounding_box({0, 0, 0, 0}) {}

Env::Env(std::vector<Wall> walls, std::string name) : walls(walls), name(name) {
    bounding_box = {
        .min_x = walls[0].get_bounding_box_coords().min_x,
        .max_x = walls[0].get_bounding_box_coords().max_x,
        .min_y = walls[0].get_bounding_box_coords().min_y,
        .max_y = walls[0].get_bounding_box_coords().max_y,
    };
    for (int i = 1; i < walls.size(); i++) {
        if (walls[i].get_bounding_box_coords().min_x < bounding_box.min_x) {
            bounding_box.min_x = walls[i].get_bounding_box_coords().min_x;
        }
        if (walls[i].get_bounding_box_coords().max_x > bounding_box.max_x) {
            bounding_box.max_x = walls[i].get_bounding_box_coords().max_x;
        }
        if (walls[i].get_bounding_box_coords().min_y < bounding_box.min_y) {
            bounding_box.min_y = walls[i].get_bounding_box_coords().min_y;
        }
        if (walls[i].get_bounding_box_coords().max_y > bounding_box.max_y) {
            bounding_box.max_y = walls[i].get_bounding_box_coords().max_y;
        }
    }
}

Env::Env(std::list<eig::Matrix<double, 2, eig::Dynamic>> obstacles, std::string name) {
    std::vector<Wall> tmp;
    for (eig::Matrix<double, 2, eig::Dynamic> const &obs : obstacles) {
        tmp.emplace_back(obs.col(0), obs.col(obs.cols() - 1));
        for (int col_idx = 1; col_idx < obs.cols(); col_idx++) {
            tmp.emplace_back(obs.col(col_idx), obs.col(col_idx - 1));
        }
    }
    Env(tmp, name);
}


Env Env::ten_by_ten = Env(
    {
        Wall(-eig::Vector2d::Ones(), -eig::Vector2d::UnitY() + eig::Vector2d::UnitX() * 9),
        Wall(-eig::Vector2d::UnitY() + eig::Vector2d::UnitX() * 9, eig::Vector2d::UnitY() * 9 + eig::Vector2d::UnitX() * 9),
        Wall(eig::Vector2d::UnitY() * 9 + eig::Vector2d::UnitX() * 9, -eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9),
        Wall(-eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9, -eig::Vector2d::Ones())
    },
    "ten_by_ten"
    );

Env Env::stripa_short = Env(
    {
        Wall(-eig::Vector2d::UnitX() - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 20 - eig::Vector2d::UnitY()),
        Wall(eig::Vector2d::UnitX() * 20 - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 2),
        Wall(eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 2, eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 2),
        Wall(eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 2, eig::Vector2d::UnitX() * 22 - eig::Vector2d::UnitY()),
        Wall(eig::Vector2d::UnitX() * 22 - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 30 - eig::Vector2d::UnitY()),

        Wall(eig::Vector2d::UnitX() * 40 - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 40 + eig::Vector2d::UnitY() * 9),

        Wall(eig::Vector2d::UnitX() * 40 + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 9),
        Wall(eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 6),
        Wall(eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 6, eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 6),
        Wall(eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 6, eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 9),
        Wall(eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 9, -eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9),

        Wall(-eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9, -eig::Vector2d::UnitX() - eig::Vector2d::UnitY())
    },
    "stripa_short"
);

Env Env::stripa = Env(
    {
        Wall(eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY()*(-1)),
        Wall(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() *(-1 + 3)),
        Wall(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3)),
        Wall(eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY()*(-1)),
        Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2)) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1)),
        Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() *(-1 + 3)),
        Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3)),
        Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY()*(-1)),
        Wall(eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2)) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1)),
        
        Wall(eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9),

        Wall(eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2)) + eig::Vector2d::UnitY() * 9),
        Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3)),
        Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * (9 - 3)),
        Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9),
        Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * ((-1 + 20 + 2)) + eig::Vector2d::UnitY() * 9),
        Wall(eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3)),
        Wall(eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * (9 - 3)),
        Wall(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * 9),
        Wall(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY() * 9),

        Wall(eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY()*(-1))
    },
    "stripa"
);
