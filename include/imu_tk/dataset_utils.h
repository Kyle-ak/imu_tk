#pragma once

#include <vector>

#include "imu_tk/base.h"

namespace imu_tk
{
enum TimestampUnit
{
  TIMESTAMPUNIT_SEC = 1,
  TIMESTAMPUNIT_MSEC = 1000,
  TIMESTAMPUNIT_USEC = 1000000,
  TIMESTAMPUNIT_NSEC = 1000000000
};

template <typename _T> 
  void importAsciiData( const char *filename,
                        std::vector< TriadData<_T> > &samples, 
                        TimestampUnit unit = TIMESTAMPUNIT_USEC );
template <typename _T> 
  void importAsciiData( const char *filename,
                        std::vector< TriadData<_T> > &samples0,
                        std::vector< TriadData<_T> > &samples1, 
                        TimestampUnit unit = TIMESTAMPUNIT_USEC );
template <typename _T> 
  void importAsciiData( const char *filename,
                        std::vector< TriadData<_T> > &samples0,
                        std::vector< TriadData<_T> > &samples1,
                        std::vector< TriadData<_T> > &samples2, 
                        TimestampUnit unit = TIMESTAMPUNIT_USEC ); 
}