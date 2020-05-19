# Set the minimum version of cmake required to build this project
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
include_guard(GLOBAL)

PROJECT(kabsch)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

add_executable(kabsch main.cpp)
target_include_directories(kabsch PUBLIC
                            ${EIGEN3_INCLUDE_DIR}
                            )
