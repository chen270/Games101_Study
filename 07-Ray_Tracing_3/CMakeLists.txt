cmake_minimum_required(VERSION 3.10)

project(07_RayTracing_3)
SET(TARGET "07_RayTracing_3")

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DDIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/")
include_directories(${THIRD_PARTY_DIR})

FILE(GLOB SRC_LIST "./src/*.cpp" "./src/*.hpp")
SOURCE_GROUP("source" FILES ${SRC_LIST})

FILE(GLOB  HEAD_LIST "./src/*.h")
SOURCE_GROUP("include" FILES ${HEAD_LIST})

SET(ALL_SOURCES
${SRC_LIST}
${HEAD_LIST}
)

add_executable(${TARGET} ${ALL_SOURCES})

if (MSVC)
#     target_compile_options(${TARGET} PUBLIC -Wall -pedantic -fsanitize=undefined)

     # Set the startup project .
    if (NOT ${CMAKE_VERSION} VERSION_LESS "3.6.0")
          set_property(DIRECTORY ${MSVC_DIR} PROPERTY VS_STARTUP_PROJECT ${TARGET})
    endif ()

    # copy dll
    # file(COPY "${OpenCV_DIR}/x64/vc16/bin" DESTINATION "${MSVC_DIR}")
else()
#     target_compile_options(${TARGET} PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined)
endif (MSVC)

set(INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${INSTALL_DIR}")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${INSTALL_DIR}")