#pragma once

#include <vector>

#include "imu_tk/base.h"

namespace imu_tk
{
enum TimestampUnit
{
  TIMESTAMP_UNIT_SEC = 1,
  TIMESTAMP_UNIT_MSEC = 1000,
  TIMESTAMP_UNIT_USEC = 1000000,
  TIMESTAMP_UNIT_NSEC = 1000000000
};

template <typename _T> 
  void importAsciiData( const char *filename,
                        std::vector< TriadData<_T> > &samples, 
                        TimestampUnit unit = TIMESTAMP_UNIT_USEC );
template <typename _T> 
  void importAsciiData( const char *filename,
                        std::vector< TriadData<_T> > &samples0,
                        std::vector< TriadData<_T> > &samples1, 
                        TimestampUnit unit = TIMESTAMP_UNIT_USEC );
template <typename _T> 
  void importAsciiData( const char *filename,
                        std::vector< TriadData<_T> > &samples0,
                        std::vector< TriadData<_T> > &samples1,
                        std::vector< TriadData<_T> > &samples2, 
                        TimestampUnit unit = TIMESTAMP_UNIT_USEC ); 
}