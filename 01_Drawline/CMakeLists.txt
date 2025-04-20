cmake_minimum_required(VERSION 3.10)
project(Drawline)

set(CMAKE_CXX_STANDARD 17)

add_executable(Drawline
    main.cpp
    model.cpp
    tgaimage.cpp
)

target_include_directories(Drawline PRIVATE ${CMAKE_SOURCE_DIR})

file(COPY ${PROJECT_SOURCE_DIR}/obj/ DESTINATION ${PROJECT_BINARY_DIR}/../obj/)