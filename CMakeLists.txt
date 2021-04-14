cmake_minimum_required(VERSION 3.0)

project(app)

find_package(sciplot)

add_executable(app main.cpp sim.cpp env.cpp range_ray.cpp)

target_link_libraries(app sciplot::sciplot)