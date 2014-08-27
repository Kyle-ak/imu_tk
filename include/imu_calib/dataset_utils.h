#pragma once

#include <vector>

#include "imu_calib/common.h"

namespace imu_calib
{
  void importMatlabData( const char *filename,
                         std::vector< imu_calib::TriadData > &data );
  void importMatlabData( const char *filename,
                         std::vector< imu_calib::TriadData > &data0,
                         std::vector< imu_calib::TriadData > &data1 );
  void importMatlabData( const char *filename,
                         std::vector< imu_calib::TriadData > &data0,
                         std::vector< imu_calib::TriadData > &data1,
                         std::vector< imu_calib::TriadData > &data2 ); 
}