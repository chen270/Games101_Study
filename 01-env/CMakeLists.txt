cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

# find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/../third_party/")
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (Transformation main.cpp)
