cmake_minimum_required(VERSION 3.10)
project(06_RayTracing)
SET(TARGET "06_RayTracing")

set(CMAKE_CXX_STANDARD 17)

set(THIRD_PARTY_DIR "${PROJECT_SOURCE_DIR}/../third_party/")
add_definitions(-DDIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/")
include_directories(${THIRD_PARTY_DIR})


add_executable(${TARGET} main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)


if (MSVC)
    target_compile_options(${TARGET} PUBLIC -Wall -pedantic -fsanitize=undefined)

     # Set the startup project .
    if (NOT ${CMAKE_VERSION} VERSION_LESS "3.6.0")
          set_property(DIRECTORY ${MSVC_DIR} PROPERTY VS_STARTUP_PROJECT ${TARGET})
    endif ()

    # copy dll
    # file(COPY "${OpenCV_DIR}/x64/vc16/bin" DESTINATION "${MSVC_DIR}")
else()
    target_compile_options(${TARGET} PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined)
endif (MSVC)
