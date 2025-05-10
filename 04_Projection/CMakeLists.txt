cmake_minimum_required(VERSION 3.10)
project(Projection)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_executable(Projection
    main.cpp
    model.cpp
    tgaimage.cpp
    rasterizer.cpp
)

target_include_directories(Projection PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

file(COPY ${PROJECT_SOURCE_DIR}/obj/ DESTINATION ${PROJECT_BINARY_DIR}/../obj/)