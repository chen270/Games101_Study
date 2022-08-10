cmake_minimum_required(VERSION 3.10)
project(Rasterizer)
SET(TARGET "Rasterizer")

set(THIRD_PARTY_DIR "${PROJECT_SOURCE_DIR}/../third_party/")

set(OpenCV_DIR "${THIRD_PARTY_DIR}/opencv-3.4.13")
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-DCV_IGNORE_DEBUG_BUILD_GUARD)

include_directories(${THIRD_PARTY_DIR})
include_directories("${OpenCV_DIR}/include")

set(MSVC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")

add_executable(${TARGET} main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(${TARGET} ${OpenCV_LIBRARIES})

set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${MSVC_DIR}/bin")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${MSVC_DIR}/bin")

if (MSVC)
     # Set the startup project .
    if (NOT ${CMAKE_VERSION} VERSION_LESS "3.6.0")
          set_property(DIRECTORY ${MSVC_DIR} PROPERTY VS_STARTUP_PROJECT ${TARGET})
    endif ()

    # copy dll
    file(COPY "${OpenCV_DIR}/x64/vc16/bin" DESTINATION "${MSVC_DIR}")

    # add_custom_command(TARGET MyTest POST_BUILD        # Adds a post-build event to MyTest
    # COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
    #     "${PROJECT_SOURCE_DIR}/libs/test.dll"      # <--this is in-file
    #     $<TARGET_FILE_DIR:MyTest>)                 # <--this is out-file path
endif (MSVC)