cmake_minimum_required(VERSION 3.13)
project(FDX_Arrow)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

add_library(FDX_Arrow src/FDX_Geo.cpp src/FDX_Vct.cpp)