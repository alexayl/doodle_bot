cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_navigation)

target_sources(app PRIVATE test_navigation.cpp
                           ../../app/src/navigation.cpp
                           ../../app/src/instruction_parser.cpp)
target_include_directories(app PRIVATE ../../include)