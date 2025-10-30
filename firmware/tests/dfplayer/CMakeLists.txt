cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dfplayer_test)

# Add the source files
target_sources(app PRIVATE dfplayer_test.cpp)

# Add driver sources
target_sources(app PRIVATE ../../drivers/dfplayer/dfplayer.c)

# Add include directories
target_include_directories(app PRIVATE ../../include)