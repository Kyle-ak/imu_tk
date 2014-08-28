project(imu_toolkit)

cmake_minimum_required (VERSION 2.8) 
cmake_policy(SET CMP0015 NEW)

# Change to "Debug" to enable debug symbols
set(CMAKE_BUILD_TYPE "Debug")

message("${CMAKE_CXX_FLAGS_RELEASE}")

find_package(Boost REQUIRED)  
find_package(Eigen REQUIRED)
find_package(Ceres REQUIRED)

include_directories(./include
                    /usr/include
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR}
                    ${CERES_INCLUDE_DIRS})

aux_source_directory( src IMU_TOOLKIT_SRC )

add_library(imu_toolkit STATIC ${IMU_TOOLKIT_SRC})
set_target_properties(imu_toolkit PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_executable(test_imu_calib apps/test_imu_calib.cpp)
target_link_libraries( test_imu_calib
                       imu_toolkit
                       ${CERES_LIBRARIES})
                       
set_target_properties( test_imu_calib PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)