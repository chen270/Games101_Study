cmake_minimum_required(VERSION 3.10)
project(03_Rasterizer)
SET(TARGET "03_Rasterizer")

set(THIRD_PARTY_DIR "${PROJECT_SOURCE_DIR}/../third_party/")

set(OpenCV_DIR "${THIRD_PARTY_DIR}/opencv-3.4.13")
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DCV_IGNORE_DEBUG_BUILD_GUARD)
add_definitions(-DDIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/")

include_directories(${THIRD_PARTY_DIR})
include_directories("${OpenCV_DIR}/include")

set(MSVC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")


add_executable(${TARGET} main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(${TARGET} ${OpenCV_LIBRARIES})
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)

set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${MSVC_DIR}/bin")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${MSVC_DIR}/bin")

if (MSVC)
     # Set the startup project .
    if (NOT ${CMAKE_VERSION} VERSION_LESS "3.6.0")
          set_property(DIRECTORY ${MSVC_DIR} PROPERTY VS_STARTUP_PROJECT ${TARGET})
    endif ()

    # copy dll
    file(COPY "${OpenCV_DIR}/x64/vc16/bin" DESTINATION "${MSVC_DIR}")
endif (MSVC)