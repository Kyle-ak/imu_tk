#pragma once

#include <vector>

#include "imu_tk/common.h"

namespace imu_tk
{
  void importMatlabData( const char *filename,
                         std::vector< imu_tk::TriadData > &data );
  void importMatlabData( const char *filename,
                         std::vector< imu_tk::TriadData > &data0,
                         std::vector< imu_tk::TriadData > &data1 );
  void importMatlabData( const char *filename,
                         std::vector< imu_tk::TriadData > &data0,
                         std::vector< imu_tk::TriadData > &data1,
                         std::vector< imu_tk::TriadData > &data2 ); 
}