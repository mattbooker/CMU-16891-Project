cmake_minimum_required(VERSION 2.8)
project(CMU-16891-Project)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(CASADI REQUIRED)
include_directories(${CASADI_INCLUDE_DIR} include/)
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

find_package(Eigen3 REQUIRED)

# Optimization flags
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3")

# add_executable(MultiAgentSolver src/main.cpp src/ShapeDecomposer.cpp src/Quadcopter.cpp src/TrajectoryOptimizer.cpp src/DoubleIntegrator.cpp src/CBSSolver.cpp)
add_executable(MultiAgentSolver src/main.cpp src/ShapeDecomposer.cpp src/ShapeDecomposerSphere.cpp src/Quadcopter.cpp src/TrajectoryOptimizer.cpp src/DoubleIntegrator.cpp src/CBSSolver.cpp)
target_compile_features(MultiAgentSolver PRIVATE cxx_std_17)
target_link_libraries(MultiAgentSolver ${CASADI_LIBRARIES} Eigen3::Eigen)