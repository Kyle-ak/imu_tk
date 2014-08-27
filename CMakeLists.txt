project(imu_calibration)

cmake_minimum_required (VERSION 2.8) 
cmake_policy(SET CMP0015 NEW)

# Change to "Debug" to enable debug symbols
set(CMAKE_BUILD_TYPE "Release")

set(STD_FLAGS_DEBUG "-Wall -g -D__STDC_CONSTANT_MACROS -pthread")
set(STD_FLAGS_RELEASE "-Wall -D__STDC_CONSTANT_MACROS -pthread -export-dynamic -march=nocona -mfpmath=sse -ftree-vectorize -fextended-identifiers -O3")