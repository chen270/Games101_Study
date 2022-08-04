cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(THIRD_PARTY_DIR "${PROJECT_SOURCE_DIR}/../third_party/")

set(OpenCV_DIR "${THIRD_PARTY_DIR}/opencv-3.4.13")
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)


include_directories(${THIRD_PARTY_DIR})
include_directories("${OpenCV_DIR}/include")

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
