project(imu_calibration)

cmake_minimum_required (VERSION 2.8) 
cmake_policy(SET CMP0015 NEW)

# Change to "Debug" to enable debug symbols
set(CMAKE_BUILD_TYPE "Release")

set(STD_FLAGS_DEBUG "-Wall -g -D__STDC_CONSTANT_MACROS -pthread")
set(STD_FLAGS_RELEASE "-Wall -D__STDC_CONSTANT_MACROS -pthread -export-dynamic -march=nocona -mfpmath=sse -ftree-vectorize -fextended-identifiers -O3")

find_package(Eigen REQUIRED)
find_package(Ceres REQUIRED)

include_directories(./include
                    /usr/include
                    ${EIGEN_INCLUDE_DIR}
                    ${CERES_INCLUDE_DIRS})

aux_source_directory( src IMU_CALIBRATION_SRC )

add_library(imu_calibration STATIC ${IMU_CALIBRATION_SRC})
set_target_properties(imu_calibration PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_executable(test_imu_calib apps/test_imu_calib.cpp)
target_link_libraries( test_imu_calib
                       imu_calibration
                       ${CERES_LIBRARIES})
                       
set_target_properties( test_imu_calib PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)