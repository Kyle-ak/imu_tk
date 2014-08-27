#pragma once

#include <vector>

namespace imu_calib
{
  void importMatlabData( std::vector< imu_calib::TriadData > &data );
  void importMatlabData( std::vector< imu_calib::TriadData > &data0,
                         std::vector< imu_calib::TriadData > &data1 );
  void importMatlabData( std::vector< imu_calib::TriadData > &data0,
                         std::vector< imu_calib::TriadData > &data1,
                         std::vector< imu_calib::TriadData > &data12 );
  
}