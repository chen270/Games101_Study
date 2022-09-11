cmake_minimum_required(VERSION 3.10)

project(05_RayTracing)
SET(TARGET "05_RayTracing")

set(CMAKE_CXX_STANDARD 17)
set(THIRD_PARTY_DIR "${PROJECT_SOURCE_DIR}/../third_party/")
add_definitions(-DDIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/")
include_directories(${THIRD_PARTY_DIR})


add_executable(${TARGET} main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp)


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

target_compile_features(${TARGET} PUBLIC cxx_std_17)
target_link_libraries(${TARGET} PUBLIC -fsanitize=undefined)
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${MSVC_DIR}/bin")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${MSVC_DIR}/bin")