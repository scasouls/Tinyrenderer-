cmake_minimum_required(VERSION 3.10)
project(Zbuffer)

set(CMAKE_CXX_STANDARD 17)

add_executable(Zbuffer
    main.cpp
    model.cpp
    tgaimage.cpp
)

file(COPY ${PROJECT_SOURCE_DIR}/obj/ DESTINATION ${PROJECT_BINARY_DIR}/../obj/)