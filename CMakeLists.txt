cmake_minimum_required(VERSION 3.28)
project(neighborhood_generator)

set(CMAKE_CXX_STANDARD 17)

# Add include/rapidjson add header library

add_executable(neighborhood_generator src/main.cpp)
add_executable(test src/test.cpp)
