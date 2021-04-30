cmake_minimum_required(VERSION 3.0)

project(app)

add_executable(app main.cpp sim.cpp env.cpp range_ray.cpp plotter.cpp helper.cpp paper_compare.cpp)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(app ${PYTHON_LIBRARIES})