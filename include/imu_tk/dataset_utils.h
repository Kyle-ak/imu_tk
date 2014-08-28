#pragma once

#include <vector>

#include "imu_tk/common.h"

namespace imu_tk
{
  enum TimestampUnit
  {
    TIMESTAMPUNIT_SEC = 1,
    TIMESTAMPUNIT_MSEC = 1000,
    TIMESTAMPUNIT_USEC = 1000000,
    TIMESTAMPUNIT_NSEC = 1000000000
  };
  
  void importMatlabData( const char *filename,
                         std::vector< TriadData > &samples, 
                         TimestampUnit unit = TIMESTAMPUNIT_USEC );
  void importMatlabData( const char *filename,
                         std::vector< TriadData > &samples0,
                         std::vector< TriadData > &samples1, 
                         TimestampUnit unit = TIMESTAMPUNIT_USEC );
  void importMatlabData( const char *filename,
                         std::vector< TriadData > &samples0,
                         std::vector< TriadData > &samples1,
                         std::vector< TriadData > &samples2, 
                         TimestampUnit unit = TIMESTAMPUNIT_USEC ); 
}