cmake_minimum_required(VERSION 2.8)
project(CMU-16891-Project)
include_directories(include/ )

# Optimization flags
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3")

add_executable(MultiAgentSolver src/main.cpp src/ShapeDecomposer.cpp)
# target_link_libraries(AStarSolver ${OpenCV_LIBS} )