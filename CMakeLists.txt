project(imu_tk)

cmake_minimum_required (VERSION 2.8) 
cmake_policy(SET CMP0015 NEW)

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT DEFINED CMAKE_BUILD_TYPE) 

if(NOT DEFINED BUILD_IMU_TK_EXAMPLES)
  set(BUILD_IMU_TK_EXAMPLES "OFF")
endif(NOT DEFINED BUILD_IMU_TK_EXAMPLES)

message("${CMAKE_CXX_FLAGS_RELEASE}")

find_package(Boost REQUIRED)  
find_package(Eigen REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL 1.6 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

set (IMU_TK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
set (IMU_TK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib PARENT_SCOPE)
set (IMU_TK_LIBS cv_ext PARENT_SCOPE)

include_directories(./include
                    /usr/include
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR}
                    ${CERES_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

                   
aux_source_directory( src IMU_TK_SRC )

add_library(imu_tk STATIC ${IMU_TK_SRC})
target_link_libraries( imu_tk ${PCL_LIBRARIES} )
set_target_properties(imu_tk PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

if( BUILD_IMU_TK_EXAMPLES )

add_executable(test_imu_calib apps/test_imu_calib.cpp)
target_link_libraries( test_imu_calib
                       imu_tk
                       ${CERES_LIBRARIES})
                       
set_target_properties( test_imu_calib PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(test_phone_calib apps/test_phone_calib.cpp)
target_link_libraries( test_phone_calib
                       imu_tk
                       ${CERES_LIBRARIES})
                       
set_target_properties( test_phone_calib PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(test_integration apps/test_integration.cpp)
target_link_libraries( test_integration
                       imu_tk
                       ${CERES_LIBRARIES})
                       
set_target_properties( test_integration PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

endif( BUILD_IMU_TK_EXAMPLES )